![Static Badge](https://img.shields.io/badge/ROS-melodic-blue)
# Human_Collaboration_Manipulation_System

# 概要
・ロボット産業IoTイニシアチブ協議会が作成した，人協働マニピュレーション機能インターフェース仕様書ver1.1をもとに,人協働マニピュレーションシステムをROSで実装を行った．  
・開発したパッケージは仕様書に沿った形で利用可能になるため， 様々なROS対応のマニピュレータに適用可能．  
（一例としてROS対応マニピュレータであるMOTOMAN-GP8による検証を行った）    

# 仕様
**人協働マニピュレーションモジュール**    

| 開発言語 | C ++, Python |    
|:------:|:------:|  
| OS | Linux(Ubuntu18.04) | 
| ミドルウェア | ROS melodic |  

# システムのシナリオ
シナリオとして，工場のラインを想定してデモ動画では，行われている．
以下にハードウェア構成とその外観について載せる．
https://github.com/Yukiya-Yamamoto/Human_Collaboration_Manipulation_System/assets/118329378/b532a5dd-8f77-4a58-b221-452f2e6ff84e

動画は以下から参照
https://github.com/Yukiya-Yamamoto/Human_Collaboration_Manipulation_System/assets/118329378/11cac15f-276d-4409-b84b-b1d11332c902

**このリポジトリのクローン**
このリポジトリを自身の環境に合わせてクローンする
```sh
$ cd catkin_ws/src
$ git clone https://github.com/Yukiya-Yamamoto/Human_Collaboration_Manipulation_System
```

## 各ファイルのビルド
クローンが完了したら，ビルドをおこなう
```sh
$ cd catkin_ws
$ catkin build
```

## demo_systemの動作確認
カメラ位置などのパラメータをdemo_system/human_collabolation/launch内の
ファイルから各自変更を行う．

実行コマンドは以下の通り
```sh
$ roslaunch human_collaboration HumanCollaboration.launch
```

## システムモデル
システム間のデータのやりとりは以下の通りである
https://github.com/Yukiya-Yamamoto/Human_Collaboration_Manipulation_System/assets/118329378/ca6d8d19-4893-4509-b68e-dd27b1587703

demo_systemのシステム構成は以下の通りである
https://github.com/Yukiya-Yamamoto/Human_Collaboration_Manipulation_System/assets/118329378/4d36d575-89f0-41d2-b1e0-ad45257d27a6

## 貢献者
Yukiya Yamamoto ([Yukiya-Yamamoto](https://github.com/Yukiya-Yamamoto))
