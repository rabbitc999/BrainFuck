/**
 * @file BrainFuck.cpp
 * @brief 基于 LLVM 的 BrainFuck 编译器（C++ 实现，Qt 命名风格）
 *
 * @section 编译流程
 *   BrainFuck 源码 -> 解析/优化 -> LLVM IR (.ll) -> clang + lld -> 可执行文件
 *
 * @section architecture 后端与共享指令
 *   SourceParser 负责语法校验和语义保持的优化；两个后端共用 Instruction。
 *   interpret 直接执行该序列；IrEmitter 生成文本 IR，再由 Clang 优化及链接。
 *   交叉编译只是 LLVM 后端的目标配置，不是另一种指令执行实现。
 *
 * @section 用法
 *   BrainFuck interpret <file.bf>           解释执行优化后的指令
 *   BrainFuck run <file.bf>                 编译为本机可执行文件并运行
 *   BrainFuck compile <file.bf>             编译为本机可执行文件
 *   BrainFuck compile <file.bf> --target T  交叉编译为 T 平台的静态可执行文件
 *   BrainFuck ir <file.bf> [out.ll]         仅生成 LLVM IR
 *   BrainFuck targets                       列出支持的交叉编译目标
 *   选项: --target T（交叉目标）, --tape-size N（数据带大小, 默认 1M,
 *         支持 K/M/G 后缀）
 *
 * @section 交叉编译
 *   针对 Linux 目标（x86_64 / aarch64 / riscv64）时，编译器会生成一个
 *   freestanding 运行时（用内联汇编实现 read/write/exit 裸系统调用），
 *   因此无需目标平台的 libc，直接由 lld 静态链接出可执行文件。
 *
 * @section 语言约定
 *   - 8 位单元格，算术回绕（mod 256）
 *   - 输入结束（EOF）时单元格置 0
 *   - 数据带大小可通过 --tape-size 自定义（默认 1 MiB），静态分配
 *   - 非 BF 指令字符视为注释
 */

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <fcntl.h>
#include <io.h>

#ifdef _MSC_VER
#pragma execution_character_set("utf-8")
#endif

/**
 * @brief 用作用域管理 Windows 控制台的输出编码。
 *
 * 仅在存在控制台且代码页需要切换时修改，析构时恢复调用方状态。
 * 重定向输出仍为原始 UTF-8 字节；不改变 BrainFuck 输入流的代码页。
 * 禁止复制，避免多个对象重复恢复同一份外部状态。
 */
class ConsoleOutputEncoding {
public:
    ConsoleOutputEncoding() : previousCodePage(GetConsoleOutputCP())
    {
        if (previousCodePage != 0 && previousCodePage != CP_UTF8)
            changed = SetConsoleOutputCP(CP_UTF8) != 0;
    }

    ~ConsoleOutputEncoding()
    {
        if (changed)
            SetConsoleOutputCP(previousCodePage);
    }

    ConsoleOutputEncoding(const ConsoleOutputEncoding &) = delete;
    ConsoleOutputEncoding &operator=(const ConsoleOutputEncoding &) = delete;

private:
    UINT previousCodePage;
    bool changed = false;
};

static const char kClangName[] = "clang.exe";   ///< clang 可执行文件名（Windows）
static const char kExeSuffix[] = ".exe";        ///< 本机可执行文件扩展名
#define BFC_POPEN _popen
#define BFC_PCLOSE _pclose
#else
#include <sys/wait.h>
static const char kClangName[] = "clang";       ///< clang 可执行文件名（POSIX）
static const char kExeSuffix[] = "";            ///< 本机可执行文件扩展名
#define BFC_POPEN popen
#define BFC_PCLOSE pclose
#endif

static const char kBoundsError[] = "BrainFuck: 数据带指针越界\n";

static const long long kDefaultTapeSize = 1LL << 20; ///< 默认数据带大小：1 MiB

/**
 * @brief BrainFuck 指令的操作码。
 *
 * 解析阶段会把源字符序列归并为这些高层指令，
 * 循环指令通过 argument 字段互相指向匹配的另一半。
 */
enum class OpCode {
    Add,    ///< 单元格加法：*(ptr + offset) = (*(ptr + offset) + argument) & 0xFF
    Move,   ///< 指针移动：ptr += argument
    Out,    ///< 将当前单元格追加到输出缓冲，满缓冲或换行时刷新
    In,     ///< 输入：*ptr = getchar()，EOF 时置 0
    Loop,   ///< 循环边界：'[' 时 argument 指向匹配的 ']' 的下标，反之亦然
    MultiplyAdd, ///< 目标单元格累加源单元格乘以系数
    Set     ///< 常量赋值；当前优化器只生成清零赋值
};

/**
 * @brief 两个后端共享的优化指令，不保存源码字符位置。
 *
 * Loop 用匹配下标的方向区分开括号和闭括号；任何删改序列的优化均需重建配对。
 * 当前解析器仅为 MultiplyAdd 生成非零 offset，其目标在前置 Move 路径中校验。
 * Add/Set/In/Out 在当前生成流程中均作用于当前单元格。
 */
struct Instruction {
    OpCode opCode;          ///< 操作码
    long long argument = 0; ///< Add/Set: 数值；MultiplyAdd: 系数；Move: 位移；Loop: 配对下标
    long long offset = 0;   ///< 相对当前指针的单元格偏移，主要用于 MultiplyAdd 目标
};

/**
 * @brief 一个交叉编译目标的完整描述。
 *
 * 汇编模板中的换行由 llvmEscape() 转义为 "\0A" 后写入 IR 文本。
 */
struct TargetSpec {
    std::string triple;             ///< LLVM 目标三元组
    long long readSyscall = 0;      ///< read 系统调用号
    long long writeSyscall = 1;     ///< write 系统调用号
    long long exitSyscall = 60;     ///< exit 系统调用号
    std::string syscallTemplate;    ///< 内联汇编：系统调用指令
    std::string syscallConstraints; ///< 内联汇编：操作数约束
    std::string startTemplate;      ///< 内联汇编：_start 入口（调用 main 后 exit）
};

/**
 * @brief 将文本转义为 LLVM IR (.ll) 字符串字面量的安全形式。
 *
 * 目前处理换行、双引号与反斜杠；其余字符原样保留。
 *
 * @param text 待转义的原始文本（通常是内联汇编模板）
 * @return 转义后可直接嵌入 .ll 字符串字面量的文本
 */
static std::string llvmEscape(const std::string &text)
{
    std::string result;
    result.reserve(text.size());
    for (char c : text) {
        switch (c) {
        case '\n': result += "\\0A"; break;
        case '"':  result += "\\22"; break;
        case '\\': result += "\\5C"; break;
        default:   result += c; break;
        }
    }
    return result;
}

/**
 * @brief 解析用户输入的数据带大小字符串。
 *
 * 支持纯字节数与 K/M/G 后缀（二进制倍数：K=1024，M=1024²，G=1024³），
 * 大小写不敏感；后缀与数字之间可有可无的空格。
 *
 * @param text        用户输入，例如 "1048576"、"64K"、"2M"
 * @param size        输出参数：解析成功时填写字节数
 * @param errorMessage 输出参数：解析失败时填写错误描述
 * @return 解析成功返回 true
 */
static bool parseTapeSize(const std::string &text, long long &size,
                          std::string &errorMessage)
{
    errorMessage.clear();
    std::string numberPart;
    char suffix = 0;
    for (char c : text) {
        if (c == ' ' || c == '\t')
            continue;
        if (c >= '0' && c <= '9' && suffix == 0) {
            numberPart += c;
        } else if ((c == 'K' || c == 'k' || c == 'M' || c == 'm'
                    || c == 'G' || c == 'g') && suffix == 0) {
            suffix = static_cast<char>(c & ~0x20);  // 统一为大写
        } else {
            errorMessage = "无法识别的数据带大小: " + text;
            return false;
        }
    }
    if (numberPart.empty()) {
        errorMessage = "数据带大小缺少数字: " + text;
        return false;
    }
    long long value = 0;
    try {
        value = std::stoll(numberPart);
    } catch (...) {
        errorMessage = "数据带大小超出范围: " + text;
        return false;
    }
    long long multiplier = 1;
    switch (suffix) {
    case 'K': multiplier = 1024LL; break;
    case 'M': multiplier = 1024LL * 1024; break;
    case 'G': multiplier = 1024LL * 1024 * 1024; break;
    default: break;
    }
    // 先除后乘，避免检测表达式自身先发生有符号整数溢出。
    if (value > std::numeric_limits<long long>::max() / multiplier) {
        errorMessage = "数据带大小超出范围: " + text;
        return false;
    }
    value *= multiplier;
    if (value <= 0) {
        errorMessage = "数据带大小必须为正数: " + text;
        return false;
    }
    size = value;
    return true;
}

/**
 * @brief 获取全部受支持的交叉编译目标。
 *
 * @return 目标名 -> TargetSpec 的只读映射。
 *         目标名即 LLVM 三元组，例如 "x86_64-unknown-linux-gnu"。
 */
static const std::map<std::string, TargetSpec> &supportedTargets()
{
    // 约束顺序对应返回值、调用号、三个参数；~{memory} 禁止跨系统调用重排内存访问。
    // AArch64/RISC-V 的约束 0 将首参数绑定到返回寄存器，防止内联时寄存器分配出错。
    // _start 接收内核入口栈，调用 main 后直接 exit，不依赖 C 运行库的启动代码。
    static const std::map<std::string, TargetSpec> targets = {
        {"x86_64-unknown-linux-gnu",
         {"x86_64-unknown-linux-gnu", 0, 1, 60,
          "syscall", "={rax},{rax},{rdi},{rsi},{rdx},~{rcx},~{r11},~{memory}",
          "andq $$-16, %rsp\ncallq main\nmovl %eax, %edi\n"
          "movl $$60, %eax\nsyscall"}},
        {"aarch64-unknown-linux-gnu",
         {"aarch64-unknown-linux-gnu", 63, 64, 93,
          "svc #0", "={x0},{x8},0,{x1},{x2},~{memory}",
          "bl main\nmov x8, #93\nsvc #0"}},
        {"riscv64-unknown-linux-gnu",
         {"riscv64-unknown-linux-gnu", 63, 64, 93,
          "ecall", "={a0},{a7},0,{a1},{a2},~{memory}",
          "call main\nli a7, 93\necall"}},
    };
    return targets;
}

/**
 * @brief BrainFuck 源码解析器，兼做轻量级窥孔优化。
 *
 * 优化内容：
 *  1. 忽略注释后连续的 +/- 合并，移除模 256 后为零的 Add；
 *  2. 同方向的指针移动合并，保留转向以检查中途越界；
 *  3. 单元格按奇数步长变化的清零循环被替换为 Set 指令；
 *  4. 删除已知不会执行的循环、合并相邻 Set，并重建跳转下标；
 *  5. 将符合条件的线性搬运循环改写为 MultiplyAdd 与清零。
 * 更深层的优化（循环展开、死代码消除等）交给 clang 的 -O3 完成。
 */
class SourceParser {
public:
    /**
     * @brief 解析 BrainFuck 源码并输出优化后的指令序列。
     *
     * @param sourceText   完整源码文本（可含注释字符）
     * @param errorMessage 输出参数：解析失败时填写错误描述，成功时清空
     * @return 优化后的指令列表；解析失败时返回空列表
     */
    std::vector<Instruction> parse(const std::string &sourceText,
                                   std::string &errorMessage) const
    {
        std::vector<Instruction> instructions;
        std::vector<long long> loopStack;
        errorMessage.clear();

        // 先过滤注释，使被注释分隔的加减仍可合并。
        // strchr 对 NUL 也会返回命中，因此必须单独排除 NUL 源字节。
        std::string sourceCode;
        for (char c : sourceText) {
            if (std::strchr("+-<>.,[]", c) != nullptr && c != '\0')
                sourceCode += c;
        }

        const long long length = static_cast<long long>(sourceCode.size());
        long long index = 0;
        while (index < length) {
            const char c = sourceCode[index];
            if (c == '+' || c == '-') {
                long long value = 0;
                while (index < length &&
                       (sourceCode[index] == '+' || sourceCode[index] == '-')) {
                    // 每步取模，使超长加减串也不会累积到 long long 溢出。
                    value = (value + (sourceCode[index] == '+' ? 1 : 255)) % 256;
                    ++index;
                }
                if (value != 0)
                    instructions.push_back({OpCode::Add, value, 0});
            } else if (c == '>' || c == '<') {
                long long offset = 0;
                while (index < length &&
                       sourceCode[index] == c) {
                    offset += (sourceCode[index] == '>') ? 1 : -1;
                    ++index;
                }
                // 注意: Move 会持久改变指针位置, 不能折进相邻 Add 的
                // offset, 否则指针会停在原处造成语义错误; 这里只折叠
                // 连续的 </>, 其余交给 LLVM -O3 优化。
                instructions.push_back({OpCode::Move, offset, 0});
            } else if (c == '.') {
                instructions.push_back({OpCode::Out, 0, 0});
                ++index;
            } else if (c == ',') {
                instructions.push_back({OpCode::In, 0, 0});
                ++index;
            } else if (c == '[') {
                loopStack.push_back(
                    static_cast<long long>(instructions.size()));
                instructions.push_back({OpCode::Loop, -1, 0});
                ++index;
            } else if (c == ']') {
                if (loopStack.empty()) {
                    errorMessage = "多余的 ']'";
                    return {};
                }
                const long long open = loopStack.back();
                loopStack.pop_back();
                instructions.push_back({OpCode::Loop,
                                        open,
                                        0});
                instructions[open].argument =
                    static_cast<long long>(instructions.size()) - 1;
                ++index;
            } else {
                ++index;  // 其他字符视为注释
            }
        }
        if (!loopStack.empty()) {
            errorMessage = "有 " + std::to_string(loopStack.size())
                           + " 个未闭合的 '['";
            return {};
        }
        // 清零替换暂不删除元素，避免破坏尚未压缩的外层括号下标。
        // compact 清理冗余后，搬运识别才能直接使用可靠的循环范围。
        simplifyZeroLoops(instructions);
        return optimizeCopies(compact(instructions));
    }

private:
    /**
     * @brief 奇数步长与 256 互质，因此单单元格循环必定到达零。
     *
     * 三条指令全部替换为幂等的清零操作，保留外层循环的下标。
     *
     * @param instructions 待改写的指令序列（原地修改）
     */
    static void simplifyZeroLoops(std::vector<Instruction> &instructions)
    {
        const long long count = static_cast<long long>(instructions.size());
        for (long long i = 0; i < count; ++i) {
            Instruction &open = instructions[i];
            if (open.opCode != OpCode::Loop || open.argument - i != 2)
                continue;
            const Instruction &body = instructions[i + 1];
            if (body.opCode == OpCode::Add && body.offset == 0
                    && (body.argument % 2 == 1)) {
                const long long close = open.argument;
                instructions[i] = {OpCode::Set, 0, 0};
                instructions[i + 1] = {OpCode::Set, 0, 0};
                instructions[close] = {OpCode::Set, 0, 0};
            }
        }
    }

    /**
     * @brief 将线性搬运循环转换为常数次乘加，同时保留边界检查语义。
     *
     * 循环只允许 Add/Move，最终指针必须回到源单元格，源增量 d 必须为奇数。
     * 在模 256 下 d 可逆；初值 x 到零所需轮数为 -x * inverse(d)。
     * 因而目标每轮增量 c 对应一次 target += x * (-inverse(d) * c)。
     *
     * 外层循环保留零值判断，内部先走一遍原移动路径检查边界，再做乘加和清零。
     * 重复迭代的路径相同，检查一次即可；源为零时则完全跳过检查与目标访问。
     *
     * @param input 已压缩、括号已配对的指令序列
     * @return 新指令序列，所有 Loop 已重新配对；不修改 input
     */
    static std::vector<Instruction> optimizeCopies(const std::vector<Instruction> &input)
    {
        std::vector<Instruction> result;
        for (std::size_t i = 0; i < input.size(); ++i) {
            const auto &inst = input[i];
            bool valid = inst.opCode == OpCode::Loop && inst.argument > static_cast<long long>(i);
            long long position = 0;
            // 以循环入口为原点，累积各相对地址每轮的模 256 增量。
            std::map<long long, int> changes;
            std::vector<Instruction> moves;
            int sourceDelta = 0;
            if (valid) {
                for (std::size_t j = i + 1; j < static_cast<std::size_t>(inst.argument); ++j) {
                    const auto &part = input[j];
                    if (part.opCode == OpCode::Move) {
                        position += part.argument;
                        moves.push_back(part);
                    } else if (part.opCode == OpCode::Add) {
                        changes[position] = (changes[position] + part.argument) % 256;
                    } else { valid = false; break; }
                }
                const auto source = changes.find(0);
                if (source != changes.end()) sourceDelta = source->second;
                valid = valid && position == 0 && sourceDelta % 2 == 1 && !moves.empty();
            }
            if (!valid) { auto copy = inst; if (copy.opCode == OpCode::Loop) copy.argument = copy.argument > static_cast<long long>(i) ? 1 : -1; result.push_back(copy); continue; }
            // 奇数的模 256 逆元也为奇数，枚举至多 128 个候选即可找到。
            int inverse = 1;
            while ((inverse * sourceDelta) % 256 != 1) inverse += 2;
            result.push_back({OpCode::Loop, 1, 0});
            result.insert(result.end(), moves.begin(), moves.end());
            for (const auto &change : changes) {
                const int factor = (256 - (inverse * change.second) % 256) % 256;
                if (change.first != 0 && factor != 0)
                    result.push_back({OpCode::MultiplyAdd, factor, change.first});
            }
            result.push_back({OpCode::Set, 0, 0});
            result.push_back({OpCode::Loop, -1, 0});
            i = static_cast<std::size_t>(inst.argument);
        }
        // 改写过程中以 +1/-1 暂记括号方向；最终按嵌套栈恢复真实下标。
        std::vector<std::size_t> stack;
        for (std::size_t i = 0; i < result.size(); ++i) {
            if (result[i].opCode != OpCode::Loop) continue;
            if (result[i].argument > static_cast<long long>(i) || result[i].argument == 1) {
                stack.push_back(i);
            } else {
                const auto open = stack.back(); stack.pop_back();
                result[open].argument = static_cast<long long>(i);
                result[i].argument = static_cast<long long>(open);
            }
        }
        return result;
    }

    /**
     * @brief 删除确定不执行的循环及相邻冗余赋值，并重建 Loop 跳转。
     *
     * knownZero 只描述当前指针所在单元格，不追踪整条数据带。
     * 移动、输入、算术后保守地丢弃该事实；输出不改变它。
     * 循环正常退出意味着退出位置的单元格为零，即使循环改变了指针。
     *
     * @param input 清零循环已替换、括号下标仍有效的指令
     * @return 压缩后的新序列；输入保持不变
     */
    static std::vector<Instruction> compact(const std::vector<Instruction> &input)
    {
        std::vector<Instruction> output;
        std::vector<std::size_t> stack;
        bool knownZero = true; // 数据带初始化为零。
        for (std::size_t i = 0; i < input.size(); ++i) {
            const Instruction &inst = input[i];
            const bool opening = inst.opCode == OpCode::Loop
                && inst.argument > static_cast<long long>(i);
            if (opening && knownZero) {
                i = static_cast<std::size_t>(inst.argument);
                continue;
            }
            if (inst.opCode == OpCode::Set && !output.empty()
                    && output.back().opCode == OpCode::Set
                    && output.back().offset == inst.offset) {
                output.back() = inst;
                knownZero = inst.argument == 0;
                continue;
            }
            const std::size_t index = output.size();
            output.push_back(inst);
            if (opening) {
                stack.push_back(index);
                knownZero = false;
            } else if (inst.opCode == OpCode::Loop) {
                const std::size_t open = stack.back();
                stack.pop_back();
                output[open].argument = static_cast<long long>(index);
                output.back().argument = static_cast<long long>(open);
                // 循环出口的当前单元格必为零，即使循环移动了指针。
                knownZero = true;
            } else if (inst.opCode == OpCode::Set) {
                knownZero = inst.offset == 0 && inst.argument == 0;
            } else if (inst.opCode != OpCode::Out) {
                knownZero = false;
            }
        }
        return output;
    }
};

/**
 * @brief 在零初始化的数据带上直接执行共享指令序列。
 *
 * 通过 unsigned char 转换实现模 256 运算。输出以 4096 字节缓冲，
 * 在换行、输入前、正常返回及越界退出时刷新，保证提示与已有输出可见。
 *
 * @param instructions 解析器生成的合法、配对完整且已优化的指令
 * @param tapeSize 数据带字节数，调用方须先验证其为正数
 * @return 正常结束为 0；指针越界为 1
 */
static int interpret(const std::vector<Instruction> &instructions, long long tapeSize)
{
    std::vector<unsigned char> tape(static_cast<std::size_t>(tapeSize), 0);
    std::vector<unsigned char> output;
    output.reserve(4096);
    const auto flush = [&output]() {
        if (!output.empty()) std::fwrite(output.data(), 1, output.size(), stdout);
        output.clear();
        std::fflush(stdout);
    };
    long long pointer = 0;
    for (std::size_t pc = 0; pc < instructions.size(); ++pc) {
        const Instruction &inst = instructions[pc];
        switch (inst.opCode) {
        case OpCode::MultiplyAdd: {
            // 先验证偏移再相加，避免越界访问和有符号加法溢出。
            // 优化器仍须保留 Move 路径，以检查途中越界及源为零时的跳过语义。
            if (inst.offset < -pointer || inst.offset > tapeSize - 1 - pointer) {
                flush();
                std::cerr << kBoundsError;
                return 1;
            }
            const long long target = pointer + inst.offset;
            tape[target] = static_cast<unsigned char>(tape[target] + tape[pointer] * inst.argument);
            break;
        }
        case OpCode::Add:
            tape[pointer] = static_cast<unsigned char>(tape[pointer] + inst.argument);
            break;
        case OpCode::Set:
            tape[pointer] = static_cast<unsigned char>(inst.argument);
            break;
        case OpCode::Move:
            // 在移动前比较剩余空间，避免先计算 pointer + argument 造成溢出。
            if ((inst.argument > 0 && pointer > tapeSize - 1 - inst.argument)
                    || (inst.argument < 0 && pointer < -inst.argument)) {
                flush();
                std::cerr << kBoundsError;
                return 1;
            }
            pointer += inst.argument;
            break;
        case OpCode::In: {
            flush();
            const int value = std::getchar();
            tape[pointer] = value < 0 ? 0 : static_cast<unsigned char>(value);
            break;
        }
        case OpCode::Out:
            output.push_back(tape[pointer]);
            if (output.size() == 4096 || tape[pointer] == 10) flush();
            break;
        case OpCode::Loop:
            // for 的自增会跳过所指括号；回边直接进入循环体，闭括号负责重查条件。
            if (inst.argument > static_cast<long long>(pc)) {
                if (tape[pointer] == 0)
                    pc = static_cast<std::size_t>(inst.argument);
            } else if (tape[pointer] != 0) {
                pc = static_cast<std::size_t>(inst.argument);
            }
            break;
        }
    }
    flush();
    return 0;
}

/**
 * @brief LLVM IR 文本生成器。
 *
 * 把优化后的指令序列展开为完整的 LLVM IR 模块，包含：
 *  - 数据带全局变量 @tape（默认 1 MiB，可配置，零初始化）；
 *  - @main 函数（全部指令的线性展开，循环用基本块实现）；
 *  - 运行时：本机模式使用批量 write 与 getchar（链接宿主 libc）；
 *    freestanding 模式生成基于裸系统调用的完整运行时（交叉编译用）。
 *
 * 生成的是 LLVM 15+ 不透明指针（ptr）语法的文本 IR。
 */
class IrEmitter {
public:
    /**
     * @brief 构造 IR 生成器。
     *
     * @param targetTriple 目标三元组；空串表示宿主平台
     * @param freestanding true 时嵌入裸系统调用运行时（交叉编译），
     *                     false 时使用宿主 libc 的输入输出接口（本机编译）
     * @param tapeSize 数据带字节数，必须为正数
     * @pre freestanding 为 true 时，targetTriple 必须在 supportedTargets() 中。
     *      命令行入口负责校验；emitRuntime 会直接使用对应的目标描述。
     */
    explicit IrEmitter(const std::string &targetTriple = "",
                       bool freestanding = false,
                       long long tapeSize = kDefaultTapeSize)
        : m_targetTriple(targetTriple), m_freestanding(freestanding),
          m_tapeSize(tapeSize)
    {
        if (freestanding && !targetTriple.empty()) {
            const auto &targets = supportedTargets();
            const auto it = targets.find(targetTriple);
            if (it != targets.end())
                m_targetSpec = &it->second;
        }
    }

    /**
     * @brief 生成整个 LLVM IR 模块的文本。
     *
     * @param instructions 优化后的指令序列
     * @return 完整的 .ll 模块文本
     */
    std::string emit(const std::vector<Instruction> &instructions) const
    {
        std::ostringstream ir;
        ir << "; 由 BrainFuck 自动生成\n";
        if (!m_targetTriple.empty())
            ir << "target triple = \"" << m_targetTriple << "\"\n";
        ir << "\n@tape = global [" << m_tapeSize << " x i8] zeroinitializer\n\n";
        ir << emitRuntime();
        ir << emitProgram(instructions);
        return ir.str();
    }

private:
    /**
     * @brief 生成 @main 函数：把指令序列线性展开为 IR 基本块。
     *
     * 每条循环 '[' 生成三个基本块（条件头/循环体/出口），
     * 匹配的 ']' 生成回边与出口块。
     *
     * @param instructions 优化后的指令序列
     * @return @main 函数的 IR 文本
     */
    std::string emitProgram(const std::vector<Instruction> &instructions) const
    {
        std::ostringstream body;
        std::vector<std::pair<std::string, std::string>> loopStack;

        // 指针先存入栈槽，便于跨基本块读写；Clang -O3 可将其提升为 SSA 值。
        body << "define i32 @main() {\n"
             << "entry:\n"
             << "  %ptr = alloca i64\n"
             << "  store i64 0, ptr %ptr\n";
#ifdef _WIN32
        if (!m_freestanding) {
            body << "  %stdinMode = call i32 @_setmode(i32 0, i32 " << _O_BINARY << ")\n"
                 << "  %stdoutMode = call i32 @_setmode(i32 1, i32 " << _O_BINARY << ")\n"
                 << "  %stdinOk = icmp ne i32 %stdinMode, -1\n"
                 << "  %stdoutOk = icmp ne i32 %stdoutMode, -1\n"
                 << "  %streamsOk = and i1 %stdinOk, %stdoutOk\n"
                 << "  br i1 %streamsOk, label %streams.ok, label %streams.error\n"
                 << "streams.error:\n  ret i32 1\nstreams.ok:\n";
        }
#endif

        int loopIndex = 0;
        for (std::size_t i = 0; i < instructions.size(); ++i) {
            const Instruction &inst = instructions[i];
            switch (inst.opCode) {
            case OpCode::MultiplyAdd: {
                const auto source = cellPointer(body, 0);
                const auto target = cellPointer(body, inst.offset);
                const auto value = nextTemp(body), product = nextTemp(body);
                const auto previous = nextTemp(body), sum = nextTemp(body);
                body << "  " << value << " = load i8, ptr " << source << "\n"
                     << "  " << product << " = mul i8 " << value << ", " << inst.argument << "\n"
                     << "  " << previous << " = load i8, ptr " << target << "\n"
                     << "  " << sum << " = add i8 " << previous << ", " << product << "\n"
                     << "  store i8 " << sum << ", ptr " << target << "\n";
                break;
            }
            case OpCode::Add:
                emitAdd(body, inst);
                break;
            case OpCode::Move: {
                const std::string ptr = nextTemp(body);
                body << "  " << ptr << " = load i64, ptr %ptr\n";
                const std::string moved = nextTemp(body);
                body << "  " << moved << " = add i64 " << ptr << ", "
                     << inst.argument << "\n";
                // 无符号比较同时拒绝负值（表示为大无符号数）和超过右边界的值。
                const std::string valid = nextTemp(body);
                const std::string label = "move" + std::to_string(i) + ".ok";
                body << "  " << valid << " = icmp ult i64 " << moved
                     << ", " << m_tapeSize << "\n"
                     << "  br i1 " << valid << ", label %" << label
                     << ", label %bounds.error\n" << label << ":\n";
                body << "  store i64 " << moved << ", ptr %ptr\n";
                break;
            }
            case OpCode::Out: {
                const std::string cp = cellPointer(body, 0);
                const std::string byte = nextTemp(body);
                body << "  " << byte << " = load i8, ptr " << cp << "\n";
                const std::string wide = nextTemp(body);
                body << "  " << wide << " = zext i8 " << byte << " to i32\n";
                body << "  call void @bfOut(i32 " << wide << ")\n";
                break;
            }
            case OpCode::In: {
                const std::string cp = cellPointer(body, 0);
                const std::string c = nextTemp(body);
                body << "  call void @bfFlush()\n  " << c << " = call i32 @getchar()\n";
                const std::string eof = nextTemp(body);
                body << "  " << eof << " = icmp slt i32 " << c << ", 0\n";
                const std::string byte = nextTemp(body);
                body << "  " << byte << " = trunc i32 " << c << " to i8\n";
                const std::string value = nextTemp(body);
                body << "  " << value << " = select i1 " << eof
                     << ", i8 0, i8 " << byte << "\n";
                body << "  store i8 " << value << ", ptr " << cp << "\n";
                break;
            }
            case OpCode::Set: {
                const std::string cp = cellPointer(body, inst.offset);
                body << "  store i8 " << inst.argument << ", ptr "
                     << cp << "\n";
                break;
            }
            case OpCode::Loop: {
                // '[' 的 argument 指向匹配 ']' 的下标（必然大于自身下标）；
                // ']' 的 argument 指向匹配 '[' 的下标（可能为 0）。
                const std::string label =
                    "loop" + std::to_string(loopIndex++);
                if (inst.argument > static_cast<long long>(i)) {  // '['
                    body << "  br label %" << label << ".hdr\n"
                         << label << ".hdr:\n";
                    const std::string cp = cellPointer(body, 0);
                    const std::string byte = nextTemp(body);
                    body << "  " << byte << " = load i8, ptr " << cp << "\n";
                    const std::string nonzero = nextTemp(body);
                    body << "  " << nonzero << " = icmp ne i8 " << byte
                         << ", 0\n";
                    body << "  br i1 " << nonzero << ", label %" << label
                         << ".body, label %" << label << ".end\n"
                         << label << ".body:\n";
                    loopStack.emplace_back(label + ".hdr", label + ".end");
                } else {                                          // ']'
                    const auto back = loopStack.back();
                    loopStack.pop_back();
                    body << "  br label %" << back.first << "\n"
                         << back.second << ":\n";
                }
                break;
            }
            }
        }
        body << "  call void @bfFlush()\n  ret i32 0\nbounds.error:\n  call void @bfFlush()\n  call void @bfBoundsError()\n  ret i32 1\n}\n";
        return body.str();
    }

    /**
     * @brief 生成一条 Add 指令的 IR：*(ptr + offset) += argument（8 位回绕）。
     *
     * @param body 输出流（追加到当前基本块）
     * @param inst Add 指令
     */
    void emitAdd(std::ostringstream &body, const Instruction &inst) const
    {
        const std::string cp = cellPointer(body, inst.offset);
        const std::string byte = nextTemp(body);
        body << "  " << byte << " = load i8, ptr " << cp << "\n";
        // 不附加 nsw/nuw：i8 溢出是语言要求的回绕，不能被优化器视为未定义。
        const std::string result = nextTemp(body);
        body << "  " << result << " = add i8 " << byte << ", "
             << inst.argument << "\n";
        body << "  store i8 " << result << ", ptr " << cp << "\n";
    }

    /**
     * @brief 生成 tape 单元格地址的 getelementptr 表达式。
     *
     * @param body   输出流（追加到当前基本块）
     * @param offset 相对当前指针位置的单元格偏移
     * @return 指向目标单元格的指针值的临时变量名（形如 %tN）
     */
    std::string cellPointer(std::ostringstream &body, long long offset) const
    {
        const std::string ptr = nextTemp(body);
        body << "  " << ptr << " = load i64, ptr %ptr\n";
        std::string index = ptr;
        if (offset != 0) {
            const std::string shifted = nextTemp(body);
            body << "  " << shifted << " = add i64 " << ptr << ", "
                 << offset << "\n";
            index = shifted;
        }
        // 地址有效性由 Move 及搬运优化保留的检查保证；此处只计算地址。
        const std::string cp = nextTemp(body);
        body << "  " << cp << " = getelementptr [" << m_tapeSize << " x i8], "
             << "ptr @tape, i64 0, i64 " << index << "\n";
        return cp;
    }

    /**
     * @brief 生成运行时部分。
     *
     * - 本机模式：使用宿主批量写入函数与 getchar；
     * - freestanding 模式：生成 @bfSyscall（内联汇编裸系统调用）、
     *   缓冲输出、@getchar（read(0,...)）以及
     *   naked 的 @_start 入口（按目标入口约定调用 main，再 exit）。
     *   单字节 read 的非正返回统一映射为 EOF；输入路径不重试 EINTR。
     *
     * @return 运行时的 IR 文本
     */
    std::string emitRuntime() const
    {
        std::ostringstream runtime;
        runtime << emitOutputRuntime();
        const std::string message = kBoundsError;
        runtime << "@bfBoundsMessage = private constant [" << message.size()
                << " x i8] c\"" << llvmEscape(message) << "\"\n"
                << "define void @bfBoundsError() {\nentry:\n";
        if (m_freestanding) {
            runtime << "  %address = ptrtoint ptr @bfBoundsMessage to i64\n"
                    << "  %written = call i64 @bfSyscall(i64 " << m_targetSpec->writeSyscall
                    << ", i64 2, i64 %address, i64 " << message.size() << ")\n";
        } else {
#ifdef _WIN32
            runtime << "  %written = call i32 @_write(i32 2, ptr @bfBoundsMessage, i32 "
#else
            runtime << "  %written = call i64 @write(i32 2, ptr @bfBoundsMessage, i64 "
#endif
                    << message.size() << ")\n";
        }
        runtime << "  ret void\n}\n\n";
        if (!m_freestanding) {
            runtime << "declare i32 @getchar()\n\n";
#ifdef _WIN32
            runtime << "declare i32 @_setmode(i32, i32)\n\n";
#endif
            return runtime.str();
        }
        const TargetSpec &spec = *m_targetSpec;
        runtime
            << "; --- freestanding 运行时: 裸系统调用实现, 无需 libc ---\n"
            << "define i64 @bfSyscall(i64 %nr, i64 %a1, i64 %a2, i64 %a3) {\n"
            << "entry:\n"
            << "  %r = call i64 asm sideeffect \""
            << llvmEscape(spec.syscallTemplate) << "\", \""
            << llvmEscape(spec.syscallConstraints)
            << "\"(i64 %nr, i64 %a1, i64 %a2, i64 %a3)\n"
            << "  ret i64 %r\n"
            << "}\n\n"
            << "define i32 @getchar() {\n"
            << "entry:\n"
            << "  %buf = alloca i8\n"
            << "  %bp = ptrtoint ptr %buf to i64\n"
            << "  %r = call i64 @bfSyscall(i64 " << spec.readSyscall
            << ", i64 0, i64 %bp, i64 1)\n"
            << "  %ok = icmp sgt i64 %r, 0\n"
            << "  %b = load i8, ptr %buf\n"
            << "  %z = zext i8 %b to i32\n"
            << "  %v = select i1 %ok, i32 %z, i32 -1\n"
            << "  ret i32 %v\n"
            << "}\n\n"
            << "define void @_start() naked {\n"
            << "entry:\n"
            << "  call void asm sideeffect \""
            << llvmEscape(spec.startTemplate) << "\", \"~{memory}\"()\n"
            << "  ret void\n"
            << "}\n\n";
        return runtime.str();
    }

    /**
     * @brief 分配一个新的 SSA 临时变量名，不向输出流写入。
     *
     * @param body 输出流（仅用于保持接口一致，此处不写入）
     * @return 形如 "%tN" 的临时变量名
     */
    static std::string nextTemp(std::ostringstream &body)
    {
        (void)body;
        return "%t" + std::to_string(s_counter++);
    }

    /**
     * @brief 生成 LLVM 程序共用的 4 KiB 输出缓冲与刷新函数。
     *
     * @bfOut 追加一个字节，满缓冲或换行时调用 @bfFlush。
     * @bfFlush 用 phi 保存已写偏移，短写后继续发送剩余字节。
     * 裸系统调用返回 -EINTR（-4）时原地重试；宿主 libc 的 -1 不作此解释。
     * 零写入或其他错误终止本轮刷新并清空缓冲，沿用不报告输出错误的行为。
     *
     * @return 全局缓冲声明、宿主写函数声明及两个函数的 IR 文本
     */
    std::string emitOutputRuntime() const
    {
        std::ostringstream out;
        out << "@bfBuffer = internal global [4096 x i8] zeroinitializer\n"
            << "@bfUsed = internal global i64 0\n";
        if (!m_freestanding) {
#ifdef _WIN32
            out << "declare i32 @_write(i32, ptr, i32)\n";
#else
            out << "declare i64 @write(i32, ptr, i64)\n";
#endif
        }
        out << R"IR(
define void @bfFlush() {
entry:
  %length = load i64, ptr @bfUsed
  br label %check
check:
  %offset = phi i64 [0, %entry], [%next, %progress], [%offset, %retry]
  %done = icmp eq i64 %offset, %length
  br i1 %done, label %end, label %write
write:
  %address = getelementptr [4096 x i8], ptr @bfBuffer, i64 0, i64 %offset
  %remaining = sub i64 %length, %offset
)IR";
        if (m_freestanding) {
            out << "  %addressInt = ptrtoint ptr %address to i64\n"
                << "  %written = call i64 @bfSyscall(i64 " << m_targetSpec->writeSyscall
                << ", i64 1, i64 %addressInt, i64 %remaining)\n";
        } else {
#ifdef _WIN32
            out << "  %count = trunc i64 %remaining to i32\n"
                << "  %result = call i32 @_write(i32 1, ptr %address, i32 %count)\n"
                << "  %written = sext i32 %result to i64\n";
#else
            out << "  %written = call i64 @write(i32 1, ptr %address, i64 %remaining)\n";
#endif
        }
        // libc 通过 errno 表示 EINTR；这里用其正常返回域之外的哨兵禁用重试分支。
        out << "  %interrupted = icmp eq i64 %written, " << (m_freestanding ? -4 : -9999) << "\n"
            << R"IR(  br i1 %interrupted, label %retry, label %test
retry:
  br label %check
test:
  %positive = icmp sgt i64 %written, 0
  br i1 %positive, label %progress, label %end
progress:
  %next = add i64 %offset, %written
  br label %check
end:
  store i64 0, ptr @bfUsed
  ret void
}
define void @bfOut(i32 %value) {
entry:
  %used = load i64, ptr @bfUsed
  %address = getelementptr [4096 x i8], ptr @bfBuffer, i64 0, i64 %used
  %byte = trunc i32 %value to i8
  store i8 %byte, ptr %address
  %next = add i64 %used, 1
  store i64 %next, ptr @bfUsed
  %full = icmp eq i64 %next, 4096
  %newline = icmp eq i8 %byte, 10
  %flush = or i1 %full, %newline
  br i1 %flush, label %send, label %end
send:
  call void @bfFlush()
  br label %end
end:
  ret void
}
)IR";
        return out.str();
    }

    static int s_counter;               ///< 进程内共享的 SSA 名称序号；当前串行生成，无并发保护
    std::string m_targetTriple;         ///< 目标三元组（空串表示宿主平台）
    bool m_freestanding = false;        ///< 是否生成 freestanding 运行时
    const TargetSpec *m_targetSpec = nullptr; ///< freestanding 模式的目标描述
    long long m_tapeSize = kDefaultTapeSize;  ///< 数据带大小（字节数）
};

int IrEmitter::s_counter = 0;

/**
 * @brief 把文本内容写入文件（覆盖式）。
 *
 * @param path    目标文件路径
 * @param content 完整文本内容
 * @return 写入成功返回 true；无法打开或写入失败返回 false
 */
static bool writeTextFile(const std::string &path, const std::string &content)
{
    std::ofstream file(path, std::ios::binary);
    if (!file.is_open())
        return false;
    file << content;
    return file.good();
}

/**
 * @brief 读取整个文件内容。
 *
 * @param path 源文件路径
 * @param content 输出参数：完整文件内容，允许为空
 * @return 成功返回 true；无法打开或读取失败返回 false
 */
static bool readFile(const std::string &path, std::string &content)
{
    std::ifstream file(path, std::ios::binary);
    if (!file.is_open())
        return false;
    std::ostringstream buffer;
    buffer << file.rdbuf();
    if (file.bad() || buffer.bad())
        return false;
    content = buffer.str();
    return true;
}

/**
 * @brief 通过系统 shell 执行命令，兼容 Windows 的带引号路径。
 *
 * cmd.exe 对以引号开头且含多个引号的命令会错误地剥除首尾引号，
 * 因此统一用 "cmd /s /c" 包裹整条命令；非 Windows 平台直接执行。
 *
 * @note 此包装只处理 cmd 引号规则，不是通用 shell 转义或参数隔离接口。
 * @param command 已拼好的完整命令行（内部可含引号）
 * @return 命令退出码
 */
static int runCommand(const std::string &command)
{
#ifdef _WIN32
    return std::system(("cmd /s /c \"" + command + "\"").c_str());
#else
    const int status = std::system(command.c_str());
    if (status == -1)
        return 1;
    if (WIFEXITED(status))
        return WEXITSTATUS(status);
    if (WIFSIGNALED(status))
        return 128 + WTERMSIG(status);
    return 1;
#endif
}

/**
 * @brief 查找 clang 可执行文件。
 *
 * 查找顺序：PATH（where/which）-> C:\Program Files\LLVM\bin。
 *
 * @return clang 的可执行路径；找不到时返回空串
 */
static std::string findClang()
{
#ifdef _WIN32
    FILE *pipe = BFC_POPEN("where clang.exe 2>nul", "r");
#else
    FILE *pipe = BFC_POPEN("which clang 2>/dev/null", "r");
#endif
    if (pipe) {
        char buffer[512] = {};
        if (fgets(buffer, sizeof(buffer), pipe)) {
            BFC_PCLOSE(pipe);
            std::string path(buffer);
            while (!path.empty()
                   && (path.back() == '\n' || path.back() == '\r'
                       || path.back() == ' '))
                path.pop_back();
            if (!path.empty())
                return path;
        } else {
            BFC_PCLOSE(pipe);
        }
    }
    const char *candidates[] = {
        "C:\\Program Files\\LLVM\\bin\\clang.exe",
        kClangName,
    };
    for (const char *candidate : candidates) {
        if (FILE *test = fopen(candidate, "rb")) {
            fclose(test);
            return candidate;
        }
    }
    return {};
}

/**
 * @brief 调用 clang 把 LLVM IR 编译为可执行文件。
 *
 * - 本机模式（targetTriple 为空）：普通编译，链接宿主 libc；
 * - 交叉模式：freestanding 静态链接（-nostdlib -static -Wl,-no-pie），
 *   使用 lld 作为链接器，产物不依赖目标平台的任何运行库。
 *
 * @param irPath       LLVM IR (.ll) 文件路径
 * @param outputPath   输出可执行文件路径
 * @param targetTriple 目标三元组；空串表示本机编译
 * @return 编译成功返回 true
 */
static bool compileWithClang(const std::string &irPath,
                             const std::string &outputPath,
                             const std::string &targetTriple)
{
    const std::string clang = findClang();
    if (clang.empty()) {
        std::cerr << "BrainFuck: 找不到 clang\n";
        return false;
    }
    std::string command = "\"" + clang + "\"";
    if (targetTriple.empty()) {
        command += " -O3 -o \"" + outputPath + "\" \"" + irPath + "\"";
    } else {
        command += " --target=" + targetTriple
                   + " -O3 -nostdlib -static -Wl,-no-pie -fuse-ld=lld"
                   + " -o \"" + outputPath + "\" \"" + irPath + "\"";
    }
    const int rc = runCommand(command);
    if (rc != 0) {
        std::cerr << "BrainFuck: clang 编译失败 (退出码 " << rc << ")\n";
        return false;
    }
    return true;
}

/**
 * @brief 打印命令行用法与支持的交叉编译目标列表。
 */
static void printTargets()
{
    std::cout << "交叉编译目标:\n";
    for (const auto &entry : supportedTargets())
        std::cout << "  " << entry.first << "\n";
}

static void printUsage()
{
    std::cout
        << "BrainFuck - 基于 LLVM 的 BrainFuck 编译器\n\n"
        << "用法:\n"
        << "  BrainFuck interpret <file.bf>           解释执行优化后的指令，无需 clang\n"
        << "  BrainFuck run <file.bf>                 编译为本机可执行文件并运行\n"
        << "  BrainFuck compile <file.bf>             编译为本机可执行文件\n"
        << "  BrainFuck compile <file.bf> --target T  交叉编译为 T 平台的可执行文件\n"
        << "  BrainFuck ir <file.bf> [out.ll]         仅生成 LLVM IR\n"
        << "  BrainFuck targets                       列出支持的交叉编译目标\n\n"
        << "选项:\n"
        << "  --target T      交叉编译目标三元组（见下方列表）\n"
        << "  --tape-size N   数据带大小，默认 1M；支持 K/M/G 后缀，如 64K、2M\n\n";
    printTargets();
}

/**
 * @brief 程序入口：解析命令行、驱动 解析 -> IR 生成 -> clang 编译 的流水线。
 *
 * @param argc 参数个数
 * @param argv 参数数组，argv[1] 为模式（interpret/run/compile/ir/targets）
 * @return 0 表示成功，非 0 表示失败；run 返回子进程退出码（信号终止为 128 + 信号编号）
 *
 * interpret 直接执行指令，不查找 Clang；ir 只写文本模块，也不启动 Clang。
 * compile/run 才调用工具链，显式 --target 使用内嵌 Linux 运行时。
 * 诊断写 stderr，解释执行的程序字节写 stdout，便于管道及二进制测试。
 */
int main(int argc, char **argv)
{
#ifdef _WIN32
    const ConsoleOutputEncoding consoleOutputEncoding;
#endif
    if (argc < 2) {
        printUsage();
        return 1;
    }
    const std::string mode = argv[1];

    if (mode == "targets") {
        printTargets();
        return 0;
    }

    if (mode != "interpret" && mode != "run" && mode != "compile" && mode != "ir") {
        std::cerr << "BrainFuck: 未知模式: " << mode << "\n";
        printUsage();
        return 1;
    }

    // 提取 --target 与 --tape-size 参数，其余参数按顺序放入 args
    std::string targetName;
    std::string tapeSizeText = "1M";
    std::vector<std::string> args;
    for (int i = 2; i < argc; ++i) {
        const std::string option = argv[i];
        if (option == "--target" || option == "--tape-size") {
            if (i + 1 >= argc || argv[i + 1][0] == '\0'
                    || std::strncmp(argv[i + 1], "--", 2) == 0) {
                std::cerr << "BrainFuck: " << option << " 缺少参数值\n";
                return 1;
            }
            const std::string value = argv[++i];
            if (option == "--target") targetName = value;
            else tapeSizeText = value;
        } else if (option.compare(0, 2, "--") == 0) {
            std::cerr << "BrainFuck: 未知选项: " << option << "\n";
            return 1;
        } else {
            args.push_back(option);
        }
    }
    long long tapeSize = kDefaultTapeSize;
    std::string tapeSizeError;
    if (!parseTapeSize(tapeSizeText, tapeSize, tapeSizeError)) {
        std::cerr << "BrainFuck: " << tapeSizeError << "\n";
        return 1;
    }

    if (args.empty()) {
        printUsage();
        return 1;
    }
    const std::string sourcePath = args[0];

    // IR 与编译模式统一校验并解析目标，后续直接复用目标描述。
    const TargetSpec *targetSpec = nullptr;
    if (!targetName.empty() && mode != "interpret") {
        const auto &targets = supportedTargets();
        const auto it = targets.find(targetName);
        if (it == targets.end()) {
            std::cerr << "BrainFuck: 不支持的目标: " << targetName
                      << " (用 'BrainFuck targets' 查看支持列表)\n";
            return 1;
        }
        targetSpec = &it->second;
    }

    // 阶段 1: 解析 + 优化
    SourceParser parser;
    std::string sourceCode;
    if (!readFile(sourcePath, sourceCode)) {
        std::cerr << "BrainFuck: 无法读取源文件: " << sourcePath << "\n";
        return 1;
    }
    std::string errorMessage;
    const std::vector<Instruction> instructions =
        parser.parse(sourceCode, errorMessage);
    if (!errorMessage.empty()) {
        std::cerr << "BrainFuck: 语法错误: " << errorMessage << "\n";
        return 1;
    }
    std::cerr << "BrainFuck: " << instructions.size() << " 条优化后指令\n";

    if (mode == "interpret") {
        if (!targetName.empty()) {
            std::cerr << "BrainFuck: 解释模式不接受 --target\n";
            return 1;
        }
#ifdef _WIN32
        // BrainFuck 的输入输出都是原始字节，禁用 CRLF 转换及 Ctrl-Z EOF。
        if (_setmode(_fileno(stdin), _O_BINARY) == -1
                || _setmode(_fileno(stdout), _O_BINARY) == -1) {
            std::cerr << "BrainFuck: 无法将标准输入输出设置为二进制模式\n";
            return 1;
        }
#endif
        return interpret(instructions, tapeSize);
    }

    // 只移除文件名的扩展名；目录中的点及隐藏文件的开头点不算扩展名。
    const auto separator = sourcePath.find_last_of("/\\");
    const auto filenameStart = separator == std::string::npos ? 0 : separator + 1;
    const auto extension = sourcePath.find_last_of('.');
    const std::string basePath = sourcePath.substr(0,
        extension != std::string::npos && extension > filenameStart
            ? extension : std::string::npos);

    // 模式: ir - 仅输出 LLVM IR
    if (mode == "ir") {
        const std::string outputPath =
            args.size() >= 2 ? args[1] : basePath + ".ll";
        IrEmitter emitter(targetName, !targetName.empty(), tapeSize);
        if (!writeTextFile(outputPath, emitter.emit(instructions))) {
            std::cerr << "BrainFuck: 无法写入: " << outputPath << "\n";
            return 1;
        }
        std::cout << "BrainFuck: 已生成 LLVM IR: " << outputPath << "\n";
        return 0;
    }

    // 模式: run / compile - 生成可执行文件（run 额外执行）
    if (mode == "run" || mode == "compile") {
        std::string outputPath;
        std::string tripleForClang;
        if (targetName.empty()) {
            outputPath = basePath + kExeSuffix;
        } else {
            tripleForClang = targetSpec->triple;
            outputPath = basePath + "-"
                         + targetName.substr(0, targetName.find('-'));
        }

        // 阶段 2: 生成 IR（交叉编译时嵌入 freestanding 运行时）
        const std::string irPath = basePath + ".ll";
        IrEmitter emitter(tripleForClang, !tripleForClang.empty(), tapeSize);
        if (!writeTextFile(irPath, emitter.emit(instructions))) {
            std::cerr << "BrainFuck: 无法写入: " << irPath << "\n";
            return 1;
        }

        // 阶段 3: clang + lld 编译链接
        if (!compileWithClang(irPath, outputPath, tripleForClang))
            return 1;
        std::cout << "BrainFuck: 已生成可执行文件: " << outputPath << "\n";

        if (mode == "run") {
            std::string executablePath = outputPath;
#ifndef _WIN32
            if (executablePath.find('/') == std::string::npos)
                executablePath = "./" + executablePath;
#endif
            return runCommand("\"" + executablePath + "\"");
        }
        return 0;
    }

    std::cerr << "BrainFuck: 未知模式: " << mode << "\n";
    printUsage();
    return 1;
}
