## 开发流程

1.创建并切换到临时开发分支
```shell
git checkout -b newBranch
```

2.提交
```shell
git add .
git commit -m "message"
```

3.将修改合并到OpenHUTB分支
```shell
# 切换到OpenHUTB分支
git checkout OpenHUTB
git merge newBranch
```

4.清理
```shell
# 合并完成使用命令删除本地分支
git checkout -b newBranch
# 推送到远程仓库（分支前的冒号表示删除）
git push origin: newBranch
```
