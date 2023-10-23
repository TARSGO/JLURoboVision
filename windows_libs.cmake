
set(WIN_GALAXY_LIB_DIR "C:/Program Files/Daheng Imaging/GalaxySDK/Samples/VC SDK/lib/x64")
set(WIN_OPENCV_CONFIG "C:/Program Files (x86)/opencv/build/OpenCVConfig.cmake")
set(WIN_EIGEN3_CONFIG "C:/Program Files (x86)/Eigen3/share/eigen3/cmake/Eigen3Config.cmake")
set(WIN_CERES_CONFIG "C:/Program Files (x86)/Ceres/lib/cmake/Ceres/CeresConfig.cmake")

# Windows库使用方法：
# 所有上面这些依赖库在编译之后 用管理员权限命令行make install都会给你装到Program Files之类的地方。
# 进去找到安装的目录（命令行里会打印出来），找到和我上面相似的cmake文件，改成你的。

# 所有的库，它们在Windows MSVC上，链接的是lib文件，而加到环境变量里的是dll文件
# 如果不喜欢每个库的路径都单独放一遍，大可以把它们都合并起来，lib和lib合并，bin和bin合并，里面都有类似的结构，
# Linux系统上就是这么干的，只不过人家有“包管理器”可以控制什么时候增删更新。

# 切记：改完之后提交代码时就不要带这个文件了！不要提交它，除非你加了新的库，这时候有必要和大家讲一声，让大家备份好自己的
