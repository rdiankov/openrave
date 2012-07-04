チュートリアル002：環境に読み込んだ物体の移動
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

 :作者: 古川誠

.. code-block:: python

  from openravepy import Environment,poseFromMatrix, with_destroy
  from numpy import eye
  from numpy.random import rand
  env = Environment()
  env.SetViewer('qtcoin')
  env.AddKinBody(env.ReadKinBodyXMLFile(filename='data/mug1.kinbody.xml'))
  env.AddKinBody(env.ReadKinBodyXMLFile(filename='data/mug2.kinbody.xml'))
  bodies = env.GetBodies()
  body1 = bodies[0]
  body1.SetTransform(eye(4))
  body2 = env.GetKinBody('mug2')
  body2.SetTransform([1,0,0,0,0,0,0])
  raw_input('キーを押すとXに+0.2移動し，移動前と移動後の変換行列を出力します．')
  tran1 = body1.GetTransform()
  print '移動前'
  print tran1
  tran1[0,3] = 0.2 
  body1.SetTransform(tran1)
  print '移動後'
  print tran1
  raw_input('キーを押すとYに+0.1移動し，移動前と移動後のポーズを出力します．')
  pose1 = poseFromMatrix (tran1)
  print '移動前'
  print 'pose: ',pose1
  pose1[5] = 0.1
  body1.SetTransform(pose1)
  print '移動後'
  print 'pose: ',pose1
  while True:
      raw_input("キーを押すとランダムに移動します．")
      tran1[0:3,3] = 0.2*(rand(3)-0.5)
      body1.SetTransform(tran1)
      print "X=%f Y=%f Z=%f"%(tran1[0,3],tran1[1,3],tran1[2,3])
      print 'pose: ',poseFromMatrix(tran1)

実行
---------

実行方法
===============

.. code-block:: bash

  openrave.py --example tutorial_002

内容説明
================

- チュートリアルを実行すると，OpenRAVEのGUIが立ち上がり，下のような画像が現れます．

  .. image:: ../../images/examples/tutorial_002_two_mugs_read_kinbody.png
    :height: 200

- いま同じ場所に２つのマグカップを読み出してしまったため，重なってしまいました．このため，青いマグカップを移動させてみましょう．\n
  物体を移動するには変換行列を使う方法と，poseを使う方法の２種類があります．まずは変換行列を使って移動させます．\n
  キーを１回押すと青いマグカップが移動し，移動前後の変換行列を出力します．

  .. image:: ../../images/examples/tutorial_002_two_mugs_move1.png
    :height: 200

- 次にposeを使って移動させます．キーを１回押すと青いマグカップが再度移動し，移動前後のposeを出力します．

  .. image:: ../../images/examples/tutorial_002_two_mugs_move2.png
    :height: 200

- この次からは，キーを押す度に青いマグカップがランダムに移動します．

  .. image:: ../../images/examples/tutorial_002_two_mugs_moverand.png
    :height: 200

解説
--------------------------------------

.. code-block:: python

  from openravepy import Environment,poseFromMatrix

- openravepyから，EnvironmentとposeFromMatrixのモジュールをインポートしています． 

.. code-block:: python

  from numpy.random import rand

- numpy.randomからrandのモジュールを読み込んでいます．後で物体の位置をランダムに生成するのに使用します．

.. code-block:: python

  env.AddKinBody(env.ReadKinBodyXMLFile(filename='data/mug1.kinbody.xml'))
  env.AddKinBody(env.ReadKinBodyXMLFile(filename='data/mug2.kinbody.xml'))

- dataフォルダからmug1.kinbody.xmlとmug2.kinbody.xmlの２つをロードし，環境に加えています．

.. code-block:: python

  bodies = env.GetBodies()
  body1 = bodies[0]

-  `Environment.GetBodies` 関数を使って環境にある全ての物体を読み出し，0番目の物体（mug1：青いマグカップ）をbody1としてインスタンスを作成しています．

.. code-block:: python

  body2 = env.GetKinBody('mug2')

- 物体の名前がわかっていれば `Environment.GetKinBody` 関数を使って直接インスタンスを作成することもできます．上の例ではmug2（赤いマグカップ）をbody2としています．

.. code-block:: python

  body2.SetTransform([1,0,0,0,0,0,0])

- 物体の位置姿勢をSetする方法はtutorial_001で説明した変換行列を用いる方法の他に，poseを用いる方法があります．\n
  poseは7つの数字からなるList（もしくはarray）で，はじめの4つの数字が回転，後の3つの数字が移動を表しています．

.. code-block:: python

  raw_input('キーを押すとXに+0.2移動し，移動前と移動後の変換行列を出力します．')

- pythonの標準関数であるraw_inputはキーボードから入力を待ちます．

.. code-block:: python

  tran1 = body1.GetTransform()
  print '移動前'
  print tran1

- 環境から `KinBody.GetTransform` 関数を使ってbody1の現在の変換行列をtran1に読み出し，移動前の変換行列を出力しています．\n
  4×4の変換行列を使った物体の回転と移動では，左上の3×3の行列が回転，右上の3×1の行列が移動を示しています．

.. code-block:: python

  tran1[0,3] = 0.2 
  body1.SetTransform(tran1)
  print '移動後'
  print tran1

- その後X軸の移動を示す変換行列に+0.2を代入し，環境に反映されて物体が移動して移動後の変換行列が出力されます．

.. code-block:: python

  移動前
  [[ 1.  0.  0.  0.]
   [ 0.  1.  0.  0.]
   [ 0.  0.  1.  0.]
   [ 0.  0.  0.  1.]]
  移動後
  [[ 1.   0.   0.   0.2]
   [ 0.   1.   0.   0. ]
   [ 0.   0.   1.   0. ]
   [ 0.   0.   0.   1. ]]

- X軸の移動は変換行列の0行3列目（tran1[0,3]）なので，そこが+0.2になっていることが確認できます．\n
  その他，Y軸の移動は1行3列目（tran1[1,3]），Z軸の移動は2行3列目（tran1[2,3]）です．

.. code-block:: python

  raw_input('キーを押すとYに+0.1移動し，移動前と移動後のポーズを出力します．')
  pose1 = poseFromMatrix (tran1)
  print '移動前'
  print 'pose: ',pose1

-  `poseFromMatrix` 関数を使って現在の変換行列tran1からpose1に変換して，移動前のposeを出力しています．

.. code-block:: python

  pose1[5] = 0.1
  body1.SetTransform(pose1)
  print '移動後'
  print 'pose: ',pose1

- その後Y軸の移動を示すposeに+0.1を代入し，環境に反映されて物体が移動して移動後のposeが出力されます．

.. code-block:: python

  移動前
  pose:  [ 1.   0.   0.   0.   0.2  0.   0. ]
  移動後
  pose:  [ 1.   0.   0.   0.   0.2  0.1   0. ]

- Y軸の移動はposeの5列目（pose1[5]）なので，そこが0.1となっていることが確認できます．\n
  その他，X軸の移動は4列目（pose1[4]），Z軸の移動は6列目（pose1[6]）です．

.. code-block:: python

  while True:
      raw_input("キーを押すとランダムに移動します．")
      tran1[0:3,3] = 0.2*(rand(3)-0.5)
      body1.SetTransform(tran1)
      print "X=%f Y=%f Z=%f"%(tran1[0,3],tran1[1,3],tran1[2,3])
      print 'pose: ',poseFromMatrix(tran1)

- 最後はrand関数を使って物体の位置をランダムに作成し移動し，現在の位置とposeを出力します．


関連関数
--------------------------------------

- :meth:`.Environment.GetBodies` , :meth:`.Environment.GetKinBody` , :meth:`.KinBody.GetTransform` , :func:`.poseFromMatrix`

関連チュートリアル
--------------------------------------

- :mod:`.tutorial_003` - 環境に読み込んだ物体の回転（回転行列）
- :mod:`.tutorial_004` - 環境に読み込んだ物体の回転（クォータニオン）

