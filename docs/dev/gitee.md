# gitee

## 解决上传出熬过100M大文件的问题

## 参考

[git上传超过100MB的大文件](https://gitee.com/yulian174/git-push-BigFiles)

1. 安装LFS

   ```bash
   git lfs install
   ```

2. 上传文件

   ```bash
   git lfs track "neuro.pdf" 
   ```

3. 添加到 .**gitattributes** 区

   ```bash
   git add .gitattributes
   ```

4.  配置 **lfs** 仓库地址

    ```bash
    git config lfs.https://gitee.com/dong_666/neuro/info/lfs.locksverify false
    ```

5.  添加到暂存区

    ```bash
    git add .
    ```

6.  将暂存区内容添加到仓库中 

    ```bash
    git commit -m "add big file"
    ```

7.  删除`./git/hooks/pre-push`文件

    ```bash
    rm .git/hooks/pre-push
    ```

8.  上传远程代码并合并

    ```bash
    git push origin master
    # 真正推送大文件
    git lfs push --all origin
    ```

!!! 注意
    需要是企业版或企业版的试用版，否则还是会出现不能上传超过100M大文件的问题。

    
