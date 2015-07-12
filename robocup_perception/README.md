#robocup_perception

#File Configuration
src/pcl_object_detector.cpp : PCLベースの複数オブジェクト検出＋オブジェクトの重心検出（平面上で実行）
src/image_object_detector.cpp : テンプレートマッチングベースの複数物体検出

script/cifar10_recognition.py : Deep learning（Cifar10)を使用した識別用スクリプト　hogehuga.com/post-409/ （参考サイト）
script/test_Client.py : 上のスクリプトのテスト用クライアント

srv/Classify.srv : cifar10_recognition.py用のサービスファイル
		　　　
		   service  string image_name
			    bool flag
			    ---
		   client   int8 result_num

object_image : 認識用画像を入れる
template_image : テンプレート画像を入れる（名前はimage_00x.jpg)
recog_file : caffeで作ったcifar10用の設定ファイルを入れる
external : AKAZEマッチング用に導入（現在未使用）
include/robocup_perception/akaze : AKAZEマッチング用に導入（現在は未使用)
tmp : 現在は使用していないSURF、AKAZEマッチングのcppを入れている
