示教轨迹输出目录
================

`launch_teach.py` 默认把录制的轨迹写到这里（`arm_link_config.json` 里
`teach.out_dir` 控制，默认值 "trajectories"，相对该配置文件所在目录）。

生成的文件形态
--------------
  <name>.json    完整轨迹：每一帧含时间戳 + 6 轴逻辑角 + 夹爪 + 元信息
  <name>.csv     同样的数据，表格形式（方便 Excel / pandas 看一眼）

两种都带 `max_step()` 和 `validate()`：
  python launch_teach.py info trajectories/<name>.json     # 看最大单步跳变、有没有非法帧
  python launch_teach.py --convert csv trajectories/<name>.json   # json -> csv

目录用途
--------
  回放   python launch_teach.py play trajectories/<name>.json --speed 0.5
  列清单 python launch_teach.py list
  看详情 python launch_teach.py info trajectories/<name>.json

注意
----
* 回放默认先走到轨迹首点（approach），再按时间轴播放；`--no-approach` 可关掉。
* **第一遍回放请用 `--speed 0.5`**，且确认臂周围没人。
* 本目录在交付包里是空的（打包时排除了本地测试录的 smoke 数据）。
* ⚠ 打包/发布时本目录内容不随包走，只保留这个说明文件。
