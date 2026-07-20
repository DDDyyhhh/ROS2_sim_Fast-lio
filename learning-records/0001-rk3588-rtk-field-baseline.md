# RK3588 RTK 现场基础已建立

用户已经实际完成 RK3588 私有 Compose 配置、CORS/UM982 现场验收和 USB 重新枚举恢复；因此本 lesson 不再从串口连接或 CORS 账号申请讲起，而是训练如何读取状态、解释证据并向领导演示。密码和设备私有配置不记录在学习记录中。

Evidence：目标机正式节点曾达到 `RTK_FIXED`、`ntrip=CONNECTED`、`global_position_trusted=true`；300 秒静态测试为 300/300 条 Fixed；USB 重新枚举后节点恢复到 `RTK_FIXED`。
