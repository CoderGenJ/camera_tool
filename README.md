# CameraTool
这个项目是关于相机的一些代码.总体目标是完成相机的标定/SFM构建marker地图/基于marker地图的多相机标定功能
- [x] 相机标定工具
- [x] marker的提取
- [ ] 相机模型
    - [x] 针孔相机
    - [ ] 鱼眼相机
- [x] PNP计算
  - [x] 基于CERES优化
  - [ ] 基于DLT计算
- [x] 基于BA的SFM构建
    - [x] 虚拟数据生成功能
    - [ ] 整体流程结果评估
- [ ] 基于marker map的多相机标定
- [ ] 在线标定
    - [ ] 基于VP的相机标定
    - [ ] 车道线LaneATT提取
# 第三方依赖
This project includes and utilizes the [AprilTag](https://github.com/AprilRobotics/apriltag) library for tag detection and pose estimation.

## AprilTag Library

The AprilTag library is an open-source software library developed by [AprilRobotics](https://github.com/AprilRobotics). The library is used for the detection of AprilTags, which are a type of fiducial marker.

### License

The AprilTag library is licensed under the BSD 2-Clause License. Below is the full license text:

BSD 2-Clause License

Copyright (C) 2013-2016, The Regents of The University of Michigan. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those of the authors and should not be interpreted as representing official policies, either expressed or implied, of the Regents of The University of Michigan.
