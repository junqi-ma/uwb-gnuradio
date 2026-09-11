# 本机 UHD / DPDK AVX-512：问题与绕过

> 主机：i7-12700（AVX2，**无 AVX-512**）
> UHD：4.6，链接了 DPDK 21
> 雷达脚本依赖 PyUHD，不依赖 C++ `UhdBurstBackend`

雷达实跑见 [`phase1/使用说明_X410_CG400自发自收雷达.md`](phase1/使用说明_X410_CG400自发自收雷达.md)。

## 1. 现象

```bash
python3 -c "import uhd"
# Illegal instruction (SIGILL), 退出码 132
```

即便 `/etc/uhd/uhd.conf` 已经改成 kernel UDP、没有 `[use_dpdk=1]`，
`import uhd` 仍然崩。Debian 的 `python3` 还默认不把
`/usr/local/lib/python3.10/site-packages` 放进 `sys.path`，不设
`PYTHONPATH` 时是 `ModuleNotFoundError: uhd`，设了才暴露 SIGILL。

## 2. 原因

1. **UHD 4.6 在链接期就依赖 DPDK**。`libuhd.so` 的 `NEEDED` 含
   `librte_eal.so.21`、`librte_ethdev.so.21` 等。加载 PyUHD 就会把这些
   `.so` 拉进来。
2. **本机 DPDK 21 按 AVX-512 编的。** `rte_srand`、
   `eth_dev_init_cb_lists` 在 **库构造函数**里执行 `vpbroadcastq %zmm`。
3. **i7-12700 桌面版没有 AVX-512**（Intel 12 代桌面关了）。构造函数一跑就
   SIGILL。这与“网卡是否走 DPDK”无关。
4. **`use_dpdk=0` 不能当开关。** 该 device arg 仍会进 EAL；本机旧
   `uhd.conf` 还曾绑到空闲的 ConnectX 口。不要再写进 `--args`。

gdb 定位（未打补丁）：

```text
rte_srand                 @ librte_eal.so.21     vpbroadcastq zmm
eth_dev_init_cb_lists     @ librte_ethdev.so.21  vpbroadcastq zmm
```

## 3. 本机分层处置（雷达路径）

| 层 | 做法 | 作用 |
|---|---|---|
| 传输 | `/etc/uhd/uhd.conf` 只用 kernel UDP；DPDK 段备份为 `uhd.conf.dpdk.bak` | 数据面不走 DPDK |
| C++ OOT | `ENABLE_UHD_BACKEND=OFF` 编 `gnuradio-uwb` | `libgnuradio-uwb.so` 不链 UHD/DPDK |
| 绑定 | 从 `gr-uwb/build/python/uwb/bindings/uwb_python*.so` 直接加载 | 不经过会拉 UHD 的已安装包 |
| PyUHD | `PYTHONPATH=/usr/local/lib/python3.10/site-packages` | Debian `python3` 能 `import uhd` |
| DPDK 副本 | `/tmp/uhd_eal_noret` 里两处 AVX-512 构造改成 `ret`，并排在 `LD_LIBRARY_PATH` 最前 | 加载时不再 SIGILL |

`x410_cg400_hrp_echo_cir.py` 的 `bootstrap_uhd_env()` 会自动设置后三项并
`exec` 一次。`/tmp/uhd_eal_noret` **重启后消失**，开机后要重建。

## 4. 重建绕过目录

```bash
python3 gr-uwb/apps/prepare_uhd_eal_noret.py
```

脚本会：

1. 从 `/usr/local/lib/x86_64-linux-gnu` 复制 UHD 需要的 DPDK 21 `.so`
2. 把 `rte_srand`、`eth_dev_init_cb_lists` 改成 `endbr64; ret`
3. 用补丁后的 `LD_LIBRARY_PATH` 试 `import uhd`

手工自检：

```bash
PYTHONPATH=/usr/local/lib/python3.10/site-packages \
LD_LIBRARY_PATH=/tmp/uhd_eal_noret \
python3 -c "import uhd; print(uhd.__file__)"
```

应打印 site-packages 路径，退出 0。不设 `LD_LIBRARY_PATH` 仍应 SIGILL。

雷达脚本检测到该目录存在就会自动用；没有该目录时仍会补 `PYTHONPATH`，
随后在 `import uhd` 处 SIGILL。

## 5. 不要做的事

- 不要在 `--args` 里加 `use_dpdk=0`。
- 不要把系统里的 `/usr/local/lib/.../librte_eal.so` 直接改掉；只改 `/tmp` 副本。
- 不要为了“能链 UHD”把本机 `ENABLE_UHD_BACKEND` 打开：C++ 后端同样会加载
  未打补丁的 DPDK。
- 不要把 DPDK `.so` 提交进 git。
- 有 AVX-512 的机器不要用这套副本。

## 6. 长期做法

本机 100GbE 已经走 kernel UDP，DPDK 没有收益。应 **重编 UHD 4.6 并关掉
DPDK**，让 `libuhd.so` 不再 `NEEDED` 任何 `librte_*`。那之后可以删掉
`/tmp/uhd_eal_noret` 和 `bootstrap` 里对它的依赖。

在那之前，雷达实机路径维持：OOT 不链 UHD + PyUHD + 补丁副本。
