# -*- coding: utf-8 -*-
"""2026-09-23 网口全指令扩展的**叙述块** + 逐命令 notes（单一来源）。

由 tools/gen_arm_config_json.py import 后写进 config/arm_config.json 的
arm_config.extension_2026_09_23；逐条字段级落在 network_protocol.commands。
手改本文件即可，改完重跑 tools/gen_arm_config_json.py。
"""

NET_EXT = {'_comment': '全指令覆盖: 网口从 14 条扩到 38 条(24 条扩展), 让 UDP 能下发控制台 124 条里的 113 条。设计基准 '
             'docs/网口指令扩展设计_2026-09-23.md。',
 '_architecture': '热路径留类型化 opcode; 长尾走【文本透传】TEXT_EXEC(0x0B) —— 执行的就是 COM11 上同一条 '
                  'ProcessCommand(), 语义零漂移, 11 条抢引脚命令的拦截天然继承。逐条加 opcode 追不上新增且会造成两份实现分叉。',
 '_four_rules': ['命令码必须 < 0x80 —— Dispatch 把 cmd>=0x80 当回包丢弃',
                 '单帧 payload <= APROTO_MAX_PAYLOAD(64) ⇒ 大数组一律【分页】',
                 '危险命令必须带 MAGIC = 0x5A5A5A5A (防误包/防扫描器误触发)',
                 '任何会阻塞主循环的命令都不上网口 —— 本工程【无看门狗】'],
 'magic': '0x5A5A5A5A',
 'text_exec_architecture': {'flow': 'TEXT_EXEC -> ArmMain_TextGate() -> ArmMain_ExecText() -> '
                                    'ProcessCommand()(与 COM11 同一条)',
                            'sink_switch': 'main.c 的 con_sink 函数指针在执行期间从 con_write_uart 切到 '
                                           'con_capture, 收进 g_text_buf[2048]; 执行完【必须】切回(否则 COM11 '
                                           '静默失声)',
                            'gate_returns': '0=放行 / 1=交互阻塞类(空行,空格开头,#,单独 c) / 2=E-Stop 中只允许 s 和 '
                                            'h',
                            'max_command_chars': 58,
                            'max_output_bytes': 2048,
                            'chunk_bytes': 58,
                            'offset_unit': '字节(不是字符!) —— 控制台中文提示是 UTF-8 字节, PC 侧必须按字节拼接最后统一 '
                                           'decode 一次, 逐页 decode 会在分页缝上拼出坏字符',
                            'excluded_pins': 'cx cp cy cz cw ps can mg mp mo ml —— '
                                             '固件接住并把原因当输出回给你(flags.bit1=0, 【不算闸门拒绝】)'},
 'deviations_from_proposal': ['GET_JOINT_DETAIL 每轴 10B(不是提案的 11B)',
                              'PARAM_LIST 每档 10B + i32 范围(不是提案的 6B/8B + i16); 提案的 i16 会静默截断',
                              'ZERO_RESET 只收 u8 flags(去掉关节号) —— 文本 zr 没有单轴变体',
                              'CENTER_CALIB / ZERO_RESET / PARAM_SAVE 统一【走文本透传】, 不另写第二份实现',
                              'CALIB_ENTER 【一律 NACK】, 设计上不可用',
                              '新增 TEXT_EXEC / TEXT_FETCH —— 一把钥匙覆盖 113 条控制台命令',
                              'GET_EVENTS(0x06) 码位保留但未实现; 用 GET_FLAGS 轮询代替'],
 'verification': {'python': 'tests/new_commands_test.py 62 项全绿; protocol selftest + 跨语言向量',
                  'js': '971 项全绿(protocol 57 / sim 127 / bridge 14 / cross_python 404 / legacy '
                        '225 / integration 104 / udp 40)',
                  'firmware': '三构建编译全过(2026-09-23): build_net 241812 B / build_can 215096 B / build_both '
                              '244772 B',
                  'not_on_hardware': '⚠ 全程仅离线验证, 未上真机。上板前先回中位 `m 180 180 180 180 180 180 15`, '
                                     '【绝不能用 M a】'},
 '_commands_canonical': '逐条字段级(含 payload/response)在 network_protocol.commands —— 本节不再重复, '
                        '只留架构与偏差说明',
 '_per_command_notes': {'0x04 GET_JOINT_DETAIL': 'u8 joint(0~5|0xFF) -> 0x84: u8 n, u8 joint0, n '
                                                 'x 10B {u16 pos_raw, i16 speed, i16 load, u8 '
                                                 'volt(0.1V), u8 temp, u8 err, u8 ok}; 6 轴 = 62B '
                                                 '<= 64; temp/err 的 0xFF = 这一拍丢读(不是 0 度/不是故障)',
                        '0x05 GET_LIMITS': '-> 0x85: 6 x 8B = 48B {u16 '
                                           'min_raw,pos_raw,max_raw,zero_raw}; 取自 Flash '
                                           '校准记录(PID_JointLimits_t); zero_raw==0 ⇒ 未做 Home '
                                           '零点校准(固件回落 180/raw 2048)',
                        '0x07 GET_INFO': '-> 0x87: u8 tag_len, char fw_tag[16], u8 build, u8 '
                                         'link, u8 nsrv = 20B; build 0=net 1=can 2=both 3=uart '
                                         '4=off; ⚠ link 是 LinkCfg_ActiveMode() 的【启动冻结值】, 不是 '
                                         'Flash 偏好',
                        '0x0A GET_FLAGS': '-> 0x8A: u8 flags(bit0 estop/bit1 traj/bit2 '
                                          'stream/bit3 corr/bit4 teach_rec/bit5 teach_play), u8 '
                                          'run_id = 2B; ⚠ 只读变量不碰总线 ⇒ 可 10~50Hz 轮询',
                        '0x0B TEXT_EXEC': 'u32 magic(无条件), char text[](NUL 结尾, <=58) -> 0x8B: '
                                          'u16 total, u8 flags, u16 off, u8 n, text[n]; flags '
                                          'bit0=输出被 2048B 截断, bit1=被安全闸门拒绝(正文是原因说明)',
                        '0x0C TEXT_FETCH': 'u16 off, u8 n(1~58) -> 0x8C: 同 0x8B 格式; 取上一条 '
                                           'TEXT_EXEC 的后续分页',
                        '0x14 MOVE_JOINT_ONE': 'u8 joint(0~5), i16 deg*10(舵机度!), u16 v*10, u16 '
                                               'a*10 = 7B -> ACK; ⚠ 先读全场快照只改这一轴 ⇒ 任一轴丢读则整条 '
                                               'ERR_READ(7)',
                        '0x17 SET_BUS_MODE': 'u8 mode(0=正常 A:AABBB / 1=全A / 2=全B) -> ACK',
                        '0x18 SERVO_INIT': 'u8 joint(0~5|0xFF) -> ACK; ⚠ 重写该轴 PID/限位/中位并 RESET, '
                                           '同文本 i',
                        '0x22 IK_QUERY': 'i16 x,y,z(mm*10), i16 pitch*10 = 8B -> 0xA2: 6 x i16 '
                                         '舵机度*10; 【只解不算】解算+限位检查+FK 回代, 不启动运动; err=3 不可达 / 4 超限位 '
                                         '/ 6 回代超差',
                        '0x32 GET_GRAVITY': '-> 0xB2: i16 K(0.001 deg/mm)',
                        '0x33 GET_CORRECTION': '-> 0xB3: u8 en, u16 thr*100, u8 max_rounds, u8 '
                                               'pending = 5B',
                        '0x36 GET_PID': 'u8 joint(0~5) -> 0xB6: u8 p, u8 i, u8 d, u8 from_eeprom '
                                        '= 4B; ⚠ 顺序就是 p/i/d(底层 SERVO_ReadPidEeprom 形参是 (p,d,i), '
                                        'API 层已换算, 不要再换); from_eeprom=0 ⇒ 读失败',
                        '0x43 PARAM_SAVE': 'u32 magic -> ACK; 等价文本 F(参数落盘)',
                        '0x44 PARAM_LIST': 'u8 offset -> 0xC4: u8 total, u8 n, n x 10B {u8 id, '
                                           'u8 type, i32 vmin*1000, i32 vmax*1000}; ⚠ 必须 '
                                           'i32(范围里有超 i16 的值, 压 i16 会静默截断); 2+6*10=62<=64; 值用 '
                                           'PARAM_READ 单取(带上会超 64B); 翻页终止 n==0 或 offset+n>=total',
                        '0x50 TEACH_LIST': 'u8 idx -> 0xD0: u8 total, u8 idx, u8 valid, char '
                                           'name[12], u32 samples = 19B; valid=0 ⇒ tombstone',
                        '0x51 TEACH_READ': 'u8 idx, u16 off, u8 n(1~5) -> 0xD1: u8 n, n x 12B(6 '
                                           'x u16 raw); 12*5=60<=64; 1000 点 = 200 请求 ≈ 0.3s @p50 '
                                           '1.61ms; ⚠ off 越界 ⇒ n=0 = 正常读完(不是错误)',
                        '0x53 TEACH_DELETE': 'u8 idx -> ACK(写 tombstone)',
                        '0x54 TEACH_COMPACT': '-> ACK(回收 tombstone 空间; 会擦 Flash, 别频繁调)',
                        '0x55 TEACH_TOTAL': '-> 0xD5: u8 nfiles, u8 capacity, u32 total_samples, '
                                            'u32 free_bytes = 10B',
                        '0x70 CENTER_CALIB': 'u32 magic -> ACK; 执行文本 `M a`(六轴推 raw 2048) ⚠ '
                                             '改量纲基准, 不可逆',
                        '0x71 CALIB_ENTER': 'u32 magic -> 【一律 NACK(ERR_PARAM)】⚠【设计上不可用, '
                                            '不是实现漏项】cmdCalibrate() 是 for(;;) 等人在本机串口敲键; 从网口进 = '
                                            '主循环永不返回, 本工程无看门狗 ⇒ 板子彻底失联。保留码位是为了让"不可用"在协议层面显式可见',
                        '0x72 ZERO_RESET': 'u8 flags(bit0=1 同时落盘, 即 `zr save`), u32 magic -> '
                                           'ACK; ⚠ 文本 zr 【没有单轴变体】(一次复位全部六轴) ⇒ 协议也不给关节号',
                        '0x73 SYS_RESET': 'u32 magic -> ACK 【之后再】复位(先回 ACK, 150ms 后 '
                                          'NVIC_SystemReset; 顺序反了上位机只会看到超时); 远程切链路完整序列: '
                                          'TEXT_EXEC("link can") -> TEXT_EXEC("link save") -> '
                                          'SYS_RESET'}}
