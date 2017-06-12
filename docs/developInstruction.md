# 切换到git说明
目前gitlab中的pointcloud_grop/autoCar仓库代码已经整理好。
后续充分使用git的强大功能，简化我们的开发工作，促进团队合作。
目前仓库里面的代码的更新时间是５月８号的。也就是说需要把这两天的开发，相应的融合到仓库中。
请大家现在把算法开发工作同步到git仓库上来。
## 目前仓库说明
* 仓库中的package有lidar_fullscan, lidar classify, lidar_msg, lidar_cluster，把相应功能都放到对应的package里面，不要随意新建一个package
* 所有数据都放在仓库更目录的data文件夹，并且相应的命令为lidar.pcap obd.txt roi.txt。目前仓库里面的数据路径都指向这个data文件夹里面。后续的所有数据文件都不要以绝对路径定义。data目录下的文件不会上传到服务器仓库。
* 所有的meg都放在lidar_msg
* 仓库中请不要出现1.0, 2.0这样的版本备份，不要做文件的备份工作，git已经帮我们做的很好了
* 有什么单独的文档（比如算法流程，算法的pdf），请及时写到放到docs中。和代码相关的（如注释）直接在程序中注明
* 本文件也放在仓库中docs, 名字为instruction.md
## 同步到git仓库的操作如下
### 备份目前开发的工作代码
比如说现在的开发工作在~/catkin_ws/，需要把相应的代码拷贝到其他地方。
比如：
``` Shell
mv  ~/catkin_ws/src ~/src_backup
```
### clone仓库代码
``` Shell
git clone http://192.168.1.20/pointcloud_group/autoCar.git　~/catkin_ws/src
```
clone完成之后，默认的代码分支为master。使用git status 查看仓库状态。可以看到代码处在master分支。
但master分支一般作为版本发布功能，我们开发推荐使用develop分支，切记不要在master上做开发。我们需要切换到develop分支，进行操作。
### 切换到develop分支
``` Shell
git checkout -b develop origin/develop
```
使用git status，查看是否处在develop分支。
### 把这两天的工作合并到仓库develop分支
把对应修改的文件替换对应的文件。git status可以看到你修改过的地方。合并之后，进行编译，测试。确保可以编译通过，运行也是正常之后，运行仓库中的clean_repository.sh脚本（清理一些不必要的文件）。
### 合并完成之后，可以push到服务器远程仓库中
``` Shell
cd ~/catkin_ws/src
git add .                    # 把修改加入到git暂存区
git commit -m "message"　　　# 把修改加到本地仓库，并要写明做了什么修改，切记不要随便写message。
git pull --rebase　　　　　　# 把远程仓库的拉取下来，如果有冲突，请解决冲突，然后重新做第５步。
git push　　　　　　　　　　 # 上传到服务器仓库
```
