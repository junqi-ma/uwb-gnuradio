# S2-B：感知前导干扰通信 payload 的影响及消除

日期：2026-10-17；状态：待实现。基础提交 93c80b3。
分支 feature/sic-payload-collision；本任务由独立 OpenCode session 执行。
先读 S2_并行开发共同约定.md，再读本文。用户已确认：SIC训练区必须允许与未知payload重叠。
本文新增通信接收端范围，优先于旧S1中“payload碰撞不在本轮范围”的历史约束。

## 1. 第一阶段边界与角色

期望信号：code10、64 SYNC、现有支持的 SFD/PHR/速率、随机PSDU的完整通信包。
干扰信号：code9、64 SYNC 的 sensing preamble-only，无 sensing SFD/PHR/payload。
两者角色与S1主场景反转：demod profile必须是comm code10，SIC模板必须是sensing code9。
不能只改标签，仍偷偷用code9解调或code10做干扰模板。
主网格998.4MS/s，K=4，estimate8_frozen，8符号训练、online_validation_reps=0。
主场景 sensing全部前导位于comm payload中，其前8符号也与随机未知payload完全重叠。
通信SYNC/SFD/PHR保持无干扰，先隔离payload问题；跨PHR/前导碰撞列未来，不自动扩大。

## 2. 最小实现与文件边界

复用现有HRP调制/完整解调、有限前导模板生成、现有SIC核心与PDU adapter。
不复用旧feature/sic-validation full-packet SIC的“干扰FCS成功才相减”门槛。
建议新增：
- gr-uwb/apps/uwb_sim_sic_payload.py：信号/角色映射/GR flowgraph/实验驱动。
- tools/sic/payload_metrics.py、verify_s2b.py：指标及独立verifier。
- testdata/sic_payload/：配置、短QA信号、payload bit映射、MATLAB对照脚本、README。
- docs/sic/S2B_感知前导干扰通信Payload开发报告.md。
不复制整套sic_preamble_sim.py/uwb_sim_preamble_sic.py；用薄wrapper调用公共功能。
设计先记录本地MATLAB与C++的PHR速率、payload速率、编码/交织、FCS所有权和边界来源。

## 3. 包几何：先证明碰撞真正落在payload

默认目标PSDU长度96 bytes（需现有profile支持；不支持就显式选择合法且足够长的长度，
不要偷偷放宽PHY能力白名单）。随机数据必须包含00/FF等合法值，不只重复同一个固定PSDU。
使用真实调制器产生包，并验证clean链逐字节PSDU相等；不能只看FCS字段。
FCS明确只追加一次：记录输入是否含FCS、modulator append开关、decoder校验对象和结果。
如果现有FCS状态不是实际校验，先澄清并补针对性QA，不能把输出存在当FCS通过。

由真实PHY几何给出：SYNC/SFD/PHR/payload/FCS的样点区间、编码尾部/成形尾部、全窗长度。
不要用“总包长减前导”猜payload起点，也不要把Ipatov符号长度当payload符号长度。
FCS是通信数据区的一部分，单独记录其是否被覆盖；首轮可限制主干扰在PSDU数据区内。
early/center/late三位置均要容纳完整干扰有限模板及路径拖尾：
设可用数据区[a,b)，干扰含尾长度D，公开guard G，令
early=a+G，center=a+floor((b-a-D)/2)，late=b-G-D；要求b-a≥D+2G。
生成器把这些位置用于注入真值，在线SIC只获得预先声明的宽候选ROI，不获得准确起点。
位置/guard/profile公开配置可确定粗ROI，另加入校准冻结的起点抖动；记录实际训练范围。
若包不够长必须拒绝配置或明确延长合法payload，不能裁掉干扰后继续声称64符号完整重叠。

完整comm包输入demodulator，输出保持样点数、时间轴、包头区不变。
只在公开许可payload处理ROI内相减，包含干扰首8符号回填与剩余56符号；
不可拿只有payload/CIR的短窗给完整包解调器。mask外逐位不变。

## 4. 未知payload重叠及真值防火墙

主实验前8训练符号看到的是comm_payload+sensing_preamble+noise，没有干净训练区。
在线估计不能先用clean通道减去payload，也不能通过已知payload生成期望波形辅助拟合。
合法公开信息仅为两者PHY profile、干扰已知码/名义长度、粗候选ROI及可见前缀。
前缀按公共ROI/最大合法jitter+fit长度确定，不按注入起点反推精确observed_end；
逐窗报告实际训练区、visible guard和是否partial，不冒称所有情况严格8符号。
公开粗ROI的获得假设必须说明：本轮验证是“给定粗候选ROI后的SIC”，不是已经证明盲检测。

QA改变训练后的IQ时，冻结模型所有字段不变；改变训练区未知payload则允许模型变化。
可增加干净训练区和oracle CFO/路径作理想诊断，但不得混入主曲线、正式验收或门控。
SIC检测/拟合失败必须旁路并保留comm解调结果，不丢包重采。

## 5. 指标与配对统计

同一个packet/noise/channel下运行clean/raw/SIC；随机payload每包独立。
每包保存：PSDU真值hash、三路接收状态/失败stage/decoded长度/PSDU匹配/FCS结果、
SIC检测/commit/bypass/模型、真实重叠比例、IQ实际残余/独立区/尾部抑制、时序。
原始干扰IQ/PSDU真值只到评分侧，不传online runtime。

两套包错误口径都输出：
- PER_FCS：未检测/未输出/超时/头失败/长度错误/FCS失败均为错误，N为全部计划有效发送包。
- PER_PSDU：未恢复期望PSDU或内容不一致均为错误，包含FCS误通过但内容错误。
按真实接收机协议说明FCS和PSDU返回是否含FCS，不允许把含/不含FCS比较错误当解调失败。
PSDU真值评分严格相等，不能从原始payload填补失败输出。

BER只在有定义且可对齐的输出bit上计算；若失败包无bits，BER_all标N/A，
同时给conditional_BER及覆盖率/擦除率，不能将失败包删除后称总BER改善。
若增加decoder诊断bit接口，不改变判决算法；优先PER，BER缺失不阻塞最小闭环。
比较raw_fail_sic_ok与raw_ok_sic_fail，后者是损伤；CFO非零/SIC旁路也必须进入分母。
PER每cell给Wilson95%CI；0/n不写0风险，接近0的曲线不能只用少量样本宣布低PER能力。
只在曲线实际跨越且单调/区间足够支持的区域报告固定PER所需SNR差；
不跨越目标就写N/A，不远距离外推dB增益。

## 6. 分阶段矩阵（先校准、再冻结验收）

B0：设计及无干扰closure，随机PSDU，先8窗后100窗高SNR baseline。
  若clean失败先定位profile/FCS/截窗，不用扩大SIC效果矩阵掩盖baseline问题。
B1：nominal SNR30/ISR10、center、offset jitter在公开预算内、CFO0/single，
  100个acceptance包；三路逐包配对、8窗以上完整归档。
  工程目标clean=100/100、sic=100/100正确；raw结果实测，不硬编码必须失败。
B2：calibration阶段用clean SNR粗扫定位PER瀑布区，冻结5个SNR点：
  至少覆盖低PER、过渡、高PER，保存实际点值；raw/SIC采用相同点，不各自移动横轴。
  acceptance主曲线：5 SNR × ISR={0,10,20} × center/CFO0/single，
  每cell至少1000包。无干扰clean可按SNR配对复用，但记录共享sample id，不能重复充样本量。
B3：边界及困难条件，选择校准固定的一个过渡SNR和SNR30，
  ISR10 × positions={early,late}，每cell500包（4cells）；
  SNR30/ISR={0,10} × CFO={-50000,+50000} × paths={single,four}，
  center，每cell500包（8cells）。four沿用原profile，记录参数。
B4：负样本 no_sensing、wrong_code11、no_overlap，各1000包；误commit≤1%，
  旁路逐位不变；主低PER条件SIC不得隐藏raw_ok_sic_fail损伤。
  可选partial overlap/跨FCS压力单列，不进入“完全payload重叠”主结果。
B5：独立verifier、报告与图。达不到效果也交完整结果，不自动调SIC核心。

若15k+包成本过高先交B1及B2每cell100包探索，必须标“探索/不足正式样本”，
保留正式1000包矩阵为OPEN，不能用探索CI冒充正式验收。无需保存每包全IQ至数十GB。

## 7. 针对性QA与回归

- 干扰只有finite SYNC，没有误带sensing SFD/PHR/payload。
- code角色、完整comm profile、FCS一次追加、随机PSDU与clean闭环。
- 训练8符号真实位于未知payload，公开可见性预算/partial处理，无真值泄漏。
- early/center/late含尾不越header，短payload拒绝；样点数/包头/mask外逐位不变。
- no_sensing/wrong_code/no_overlap旁路；训练后变更不改变完整冻结模型。
- failure计数穷举：检测/PHR/FCS/PSDU长度/timeout/队列异常等，唯一终态且总数守恒。
- 人工FCS正确但PSDU错误、成功包BER=0但大量丢包、summary分母篡改：verifier必须抓到。
- 保留S1 core/adapter、HRP mod/demod/radar回归，不通过变更旧期望掩盖缺陷。

## 8. 验收与交付

实现验收：完整包/角色/真实重叠正确、真值边界成立、逐包分母与CI正确、QA和独立复算通过。
效果验收：B1名义clean和sic均100/100目标；曲线有无改善、损伤比例、CFO困难条件逐项实报，
不预设能恢复多少dB、不用原始IQ抑制度代替PER收益。
除共同约定的manifest，还保存payload布局、FCS契约、bit定义、SNR轴校准过程及冻结配置。
图：clean/raw/SIC的PER–SNR及CI、各ISR曲线、位置/CFO/多径表、救回/损伤计数。
报告至少回答：未知payload是否污染8符号估计；SIC救回多少包/损伤多少包；
是否有可证实的PER横轴收益；失败由何阶段主导；哪些结论尚依赖候选ROI假设。
拆分生成器、SIC估计/apply、完整demod、PMT/排队、离线统计开销；不在A重负载时采正式性能。
最终提供S2B_感知前导干扰通信Payload开发报告.md、可复制README命令、QA/log、
逐包表与汇总/CI、verifier、短fixture和未实跑项。不自动进入native/硬件或修改共享核心。
