# 本机 UHD / DPDK AVX-512：问题与解决

> 主机：i7-12700（AVX2，**无 AVX-512**）
> UHD：4.6，链接了 DPDK 21
> 状态：**已解决**（DPDK 库 + libuhd 均在本机重编；旧补丁库已废弃）

DPDK 的启用与使用见 [`DPDK_X410_CG600启用.md`](DPDK_X410_CG600启用.md)。
雷达实跑见 [`phase1/使用说明_X410_CG400自发自收雷达.md`](phase1/使用说明_X410_CG400自发自收雷达.md)。

## 1. 现象（历史）

```bash
python3 -c "import uhd"
# Illegal instruction (SIGILL)，退出码 132
```

Debian 的 `python3` 默认不把 `/usr/local/lib/python3.10/site-packages` 放进
`sys.path`；不设 `PYTHONPATH` 时是 `ModuleNotFoundError: uhd`，设了才暴露
SIGILL。

## 2. 原因

1. **`libuhd.so` 在链接期就依赖 DPDK**：`NEEDED` 含 `librte_eal.so.21`、
   `librte_ethdev.so.21` 等。
2. **装机的 DPDK 21 曾在 AVX-512 机器上编**：`rte_srand`、
   `eth_dev_init_cb_lists` 在库构造函数里执行 `vpbroadcastq %zmm`，本机一加载
   就 SIGILL。i7-12700 桌面版没有 AVX-512（12 代桌面关掉了）。
3. **`libuhd.so` 自己也有 AVX-512**。UHD 对 3 个 DPDK TU 强制
   `-march=native`：
   ```
   uhd-dpdk/dpdk_common.cpp.o     -march=native
   uhd-dpdk/dpdk_io_service.cpp.o -march=native
   udp_dpdk_link.cpp.o            -march=native
   ```
   装机的 `libuhd.so` 在这几个 TU 里有 `vpbroadcastq`，**只有真正启用 DPDK
   时**（`dpdk_ctx::get()`）才崩——所以 `import uhd` 平时不崩，一用 DPDK 就崩。
4. **`use_dpdk=0` 不能当开关**：该 device arg 仍会进 EAL。不要写进 `--args`。

## 3. 解决

| 层 | 做法 |
|---|---|
| DPDK 库 | 本机重编 DPDK 21（2025-09-11），`rte_srand`/`eth_dev_init_cb_lists` 不再含 AVX-512 |
| libuhd | 本机重编 3 个 DPDK TU 并重装（见下），`dpdk_ctx::get()` 不再含 AVX-512 |
| PyUHD | `PYTHONPATH=/usr/local/lib/python3.10/site-packages`（`bootstrap_uhd_env()` 自动补） |
| 补丁库 | `/tmp/uhd_eal_noret` **已废弃**；`bootstrap_uhd_env()` 不再加载它 |

libuhd 重编/回滚命令见
[`DPDK_X410_CG600启用.md`](DPDK_X410_CG600启用.md) §4。

## 4. 不要做的事

- 不要在 `--args` 里加 `use_dpdk=0`。
- 不要提交 `libuhd` / DPDK 的二进制。
- 不要再用 `/tmp/uhd_eal_noret` 那套补丁（会盖掉本机重编的 DPDK 库）；
  `gr-uwb/apps/prepare_uhd_eal_noret.py` 仅作历史保留。
- 有 AVX-512 的机器不需要本机的重编步骤。
