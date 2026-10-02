# GitHub 发布前清单

仓库地址：[indoor-evacuation-guidance-robot](https://github.com/RenaissanceAIOT/indoor-evacuation-guidance-robot)

## 实机验收前必须核对

- [x] 使用 GitHub 用户名与 noreply 邮箱作为公开维护者信息；
- [x] README 使用实际仓库地址；
- [ ] 确认项目标题、角色和合作关系用词与证明材料一致；
- [ ] 用实测尺寸更新 Xacro 和 Nav2 footprint；
- [ ] 用现场地图坐标更新 `exits.yaml`。

## 强烈建议补充

- [ ] 仓库封面：机器人正面照片或自绘架构图，不使用设备技术资料图片；
- [ ] 60–90 秒演示视频，覆盖导航、动态避障、感知和故障停车；
- [ ] 一份脱敏 rosbag 或测试 CSV；
- [ ] 实机测试表，包含失败样本；
- [ ] 模型权重版本、许可证与 SHA-256；
- [ ] `git tag v0.1.0` 作为简历链接的稳定版本。

## GitHub 命令

仓库已创建；源码上传状态请以 GitHub 页面为准。以下供公开提交完成后从本地维护参考，请先 clone，不要重复创建同名仓库。

```bash
git clone https://github.com/RenaissanceAIOT/indoor-evacuation-guidance-robot.git
cd indoor-evacuation-guidance-robot
git add .
git commit -m "docs: add field validation evidence"
git push origin main
```

如仓库需要公开，上传前再次确认地图、rosbag、相机画面、Wi-Fi 配置和串口日志中没有个人信息、建筑敏感信息或密钥。
