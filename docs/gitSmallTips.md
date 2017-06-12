# Git使用步骤

cd 到你需要的放置项目的路径
# clone autoCar仓库
```Shell
git clone http://192.168.1.20/pointcloud_group/autoCar.git
```
现在你可以经常使用git status查看你的仓库状态
# 切换到develop分支并跟踪远程develop分支，目前推荐大家在develop分支上做开发。
```Shell
git checkout -b develop origin/develop
git submodule update --init --recursive
```
# 做完算法软件开发工作后，提交修改 到本地index
```Shell
git add .
```
# 对所做工作做一些说明，-m 之后引号内容的是说明信息
```Shell
git commit -m "add some feature"
```
# pull更新本地仓库的内容,因为其他开发人员也会修改一些代码
```Shell
git pull --rebase
```

如果有冲突，解决冲突。然后
```Shell
git add .
git rebase --continue
```
# 推送到远端服务器
```Shell
git push
```

