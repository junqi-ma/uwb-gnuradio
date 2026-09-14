# X410 CG600 (737.28 MS/s) DPDK 启用

> 设备：X410 serial 2607179，FPGA `CG_600`，UHD 4.6 链接 DPDK 21
> 主机：i7-12700（AVX2，**无 AVX-512**），20 逻辑核
> 结论：DPDK 可用；聚合 TX 从 kernel-UDP 的 ~473 MSps 提到 **737.3 MSps**

相关：[`本机UHD_DPDK_AVX512问题与绕过.md`](本机UHD_DPDK_AVX512问题与绕过.md)（历史问题，已解决）、
[`phase1/使用说明_X410_CG400自发自收雷达.md`](phase1/使用说明_X410_CG400自发自收雷达.md)。

## 1. 为什么要 DPDK

`x410_cg400_hrp_echo_cir.py` 早期在 737.28 MS/s 下 TX 满屏 `U`，
`/tmp/opencode/tx_rate_probe.py` 实测 kernel-UDP 聚合 TX 只有 ~460–473 MSps
（≈737.28 的 64%）。用 stock UHD 例子（`tx_waveforms`/`tx_bursts`）同样复现，
说明是 **UHD kernel-UDP 发送路径** 的限制，不是设备/固件。

## 2. 必须有两条链路

UHD 的 MPM 控制（RPC）**只能走 kernel UDP**，源码里没有 RPC-over-DPDK
（`rpc_client` 无 DPDK 路径）。所以 `use_dpdk=1` 时：

| 用途 | X410 侧 | 主机侧 | 传输 |
|---|---|---|---|
| 管理 / MPM RPC | RJ45 `192.168.20.133` | USB 网卡 `enx00e04d782cc0` `192.168.20.1` | kernel UDP |
| 数据 / CHDR | QSFP `192.168.10.2` | `enp1s0f1np1` `6c:b3:11:88:c0:47` `192.168.10.1/24` | **DPDK (mlx5)** |

`mgmt_addr` 必须指向一条 **DPDK 不占用** 的链路。若管理口与数据口是同一根
QSFP（`mgmt_addr == addr`），DPDK 一接管该口，RPC 回包就被 DPDK 吞掉
（`Dropping packet: No link entry in rx table`），UHD 找不到设备。X410 管理口
必须接上（RJ45 ↔ 交换机，或直连主机）。

## 3. 前置条件

1. **DPDK 库与 libuhd 都要在本机重编**（见 §4）。AVX-512 问题不只是 DPDK 库：
   UHD 自己用 `-march=native` 编 DPDK 相关的 TU，装机的 `libuhd.so` 是在
   AVX-512 机器上编的，`dpdk_ctx::get()` 里有 `vpbroadcastq`，本机一调用
   DPDK 就 SIGILL。
2. **必须 root**：mlx5 PMD 建 DevX TIS 需要 `CAP_NET_ADMIN`，DPDK 还需要
   hugetlbfs。非 root 实测失败：
   ```
   common_mlx5: Failed to create TIS using DevX
   mlx5_pci: TIS allocation failure ... Cannot allocate memory
   EAL: Couldn't get fd on hugepage file
   ```
   所以 `--use-dpdk` 要用 `sudo` 跑。
3. **hugepages**：`/proc/meminfo` 里 `HugePages_Total` 需非 0（本机 512×2M）。
   `/dev/hugepages` 归 root 即可。
4. IOMMU 已开（`/sys/kernel/iommu_groups` 非空）。

## 4. libuhd 重编（关键一步）

`/etc/uhd/uhd.conf` 与 DPDK 库都改好后，`dpdk_ctx::get()` 仍会 SIGILL，
因为 `libuhd.so` 自带 AVX-512。UHD 对这几个 TU 强制 `-march=native`：

```
# Custom flags: uhd.dir/transport/uhd-dpdk/dpdk_common.cpp.o_FLAGS   = -march=native
# Custom flags: uhd.dir/transport/uhd-dpdk/dpdk_io_service.cpp.o_FLAGS = -march=native
# Custom flags: uhd.dir/transport/udp_dpdk_link.cpp.o_FLAGS          = -march=native
```

在本机（`-march=native` = AVX2）重编这几个 TU 并重链：

```bash
cd ~/uhd/uhd/host/build
rm -f lib/CMakeFiles/uhd.dir/transport/uhd-dpdk/dpdk_common.cpp.o \
      lib/CMakeFiles/uhd.dir/transport/uhd-dpdk/dpdk_io_service.cpp.o \
      lib/CMakeFiles/uhd.dir/transport/udp_dpdk_link.cpp.o
make uhd -j"$(nproc)"
```

安装（`install_manifest.txt` 显示本 build tree 就是 `/usr/local` 的来源）：

```bash
sudo cp -a /usr/local/lib/libuhd.so.4.6.0 /usr/local/lib/libuhd.so.4.6.0.pre-avx2.bak
sudo cp ~/uhd/uhd/host/build/lib/libuhd.so.4.6.0 /usr/local/lib/libuhd.so.4.6.0
sudo ldconfig
```

自检（应为 0）：

```bash
objdump -d --disassemble=_ZN3uhd9transport4dpdk8dpdk_ctx3getEv \
  /usr/local/lib/libuhd.so.4.6.0 | grep -cE 'vpbroadcast|\{1to'
```

回滚：`sudo cp -a /usr/local/lib/libuhd.so.4.6.0.pre-avx2.bak /usr/local/lib/libuhd.so.4.6.0`。

## 5. uhd.conf

`/etc/uhd/uhd.conf`（kernel-UDP 原始版备份在 `uhd.conf.kerneludp.bak`）：

```ini
[use_dpdk=1]
dpdk_mtu=9000
dpdk_corelist=0,1
dpdk_num_mbufs=8192
dpdk_mbuf_cache_size=64

[dpdk_mac=6c:b3:11:88:c0:47]
dpdk_lcore=1
dpdk_ipv4=192.168.10.1/24
dpdk_num_desc=4096
```

- `[use_dpdk=1]` 只是命名的配置段，**仅当 device args 里有 `use_dpdk=1` 才生效**；
  不传就是 kernel UDP，可回退。
- `dpdk_mac` 是**主机**网卡 MAC（`enp1s0f1np1`）。
- `dpdk_corelist=0,1`：core 0 当 EAL master，core 1 当 NIC I/O（会被跑满）。

Mellanox 网卡 **不需要** `dpdk-devbind` 绑 vfio-pci，mlx5 PMD 与内核共存。

## 6. 使用

```bash
sudo -E python3 -u gr-uwb/apps/x410_cg400_hrp_echo_cir.py \
  --use-dpdk --mgmt-addr 192.168.20.133 \
  --preamble-length 128 --gain-tx 50 --gain-rx 60 --no-udp \
  --pulses 200 --output /some/out
```

- `--use-dpdk` 会把 `--args` 拼成
  `mgmt_addr=<mgmt>,addr=192.168.10.2,use_dpdk=1`。
- 非 root 会立刻报错并给出 `sudo` 命令；缺 `--mgmt-addr` 也会立刻报错。
- `x410_cg400_hrp_echo_cir_sweep.py` 继承同一 parser，同样支持。
- `bootstrap_uhd_env()` 现在**不再**加载旧的 `/tmp/uhd_eal_noret` 补丁库
  （DPDK 库与 libuhd 都已在本机重编，不再需要）。

## 7. 实测（tx_rate_probe.py，sc16，6 s）

| 传输 | 聚合 TX | underflow `U` |
|---|---|---|
| kernel UDP | 472.8 MSps | 160,695 |
| **DPDK** | **737.3 MSps** | **16**（仅启动瞬间） |

`short_sends=0`、`O=0`。设备在 737.28 下完全跟得上，之前 ~460 的天花板是
UHD kernel-UDP，不是 DUC/固件。

## 8. 注意

- **CIR 锁定问题是独立的，并已另行解决**：DPDK 只解决 TX 吞吐；
  CG600 的 CIR 未锁定是校准延迟默认值少了 ~0.80 µs，见
  [`phase1/测试报告_CG600_737p28_CIR锁定.md`](phase1/测试报告_CG600_737p28_CIR锁定.md)。
- 以 root 运行时 `--output` 下的文件属 root，必要时事后 `chown`。
- 不要提交 `libuhd`/DPDK 的二进制。
