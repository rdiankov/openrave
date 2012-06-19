チュートリアル003：環境に読み込んだ物体の回転（回転行列）
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

 :作者: 古川誠

.. code-block:: python

  from openravepy import Environment, rotationMatrixFromAxisAngle, axisAngleFromRotationMatrix, matrixFromAxisAngle, with_destroy
  from numpy import eye, dot, pi
  env = Environment()
  env.SetViewer('qtcoin')
  body = env.ReadKinBodyXMLFile(filename='data/mug2.kinbody.xml')
  env.AddKinBody(body)
  body.SetTransform(eye(4))
  tran = body.GetTransform()

  handles=[]
  handles.append(env.drawarrow(p1=[0.0,0.0,0.0],p2=[0.5,0.0,0.0],linewidth=0.01,color=[1.0,0.0,0.0]))
  handles.append(env.drawarrow(p1=[0.0,0.0,0.0],p2=[0.0,0.5,0.0],linewidth=0.01,color=[0.0,1.0,0.0]))
  handles.append(env.drawarrow(p1=[0.0,0.0,0.0],p2=[0.0,0.0,0.5],linewidth=0.01,color=[0.0,0.0,0.1]))

  deg = raw_input('X軸の回転角度を入力して下さい．[degree] X = ')
  if len(deg) == 0:
      deg = -45
  rot_mat = rotationMatrixFromAxisAngle([1,0,0],float(deg)*pi/180.0)
  print 'AxisAngle = ',axisAngleFromRotationMatrix(rot_mat)
  tran[0:3,0:3] = dot(rot_mat, tran[0:3,0:3])
  body.SetTransform(tran)

  P1 = dot(rot_mat, [0,0,1])
  handles.append(env.drawarrow([0.0,0.0,0.0],P1,linewidth=0.01,color=[1.0,1.0,0.0]))

  deg = raw_input('Y軸の回転角度を入力して下さい．[degree] Y = ')
  if len(deg) == 0:
      deg = 45
  rot_mat = rotationMatrixFromAxisAngle([0,1,0],float(deg)*pi/180.0)
  print 'AxisAngle = ',axisAngleFromRotationMatrix(rot_mat)
  tran[0:3,0:3] = dot(rot_mat, tran[0:3,0:3])
  body.SetTransform(tran)

  P2 = dot(rot_mat, P1)
  handles.append(env.drawarrow([0.0,0.0,0.0],P2,linewidth=0.01,color=[1.0,1.0,0.0]))

  while True:
      raw_input('キーを押すと回転しながら移動します．')
      Tdelta = matrixFromAxisAngle ([0,0,0.5])
      Tdelta[2,3] = 0.01
      tran = dot(tran, Tdelta)
      body.SetTransform(tran)

実行
--------------------------------------

実行方法
========

.. code-block:: bash

  openrave.py --example tutorial_003

内容説明
========

- チュートリアルを実行すると，OpenRAVEのGUIが立ち上がり，下のような画像が現れます．

  .. image:: ../../images/examples/tutorial_003_mug_origin.png
    :height: 200

- 画面内で矢印は座標軸を表し，RGBの色はそれぞれXYZ軸に対応しています．\n
  X軸の回転角度を入力してみましょう．矢印の向きに向かって反時計まわりがプラスの回転方向です．

  .. image:: ../../images/examples/tutorial_003_mug_rot_Xaxis.png
    :height: 200

- 例ではX軸に-45度回転させました．新しく現れた黄色の矢印はカップの上方向が回転した結果の向きを示しています．\n
  次にY軸の回転角度を入力してみましょう．

  .. image:: ../../images/examples/tutorial_003_mug_rot_Yaxis.png
    :height: 200

- 例ではY軸に+45度回転させました．黄色の矢印は先ほどと同じくカップの上方向が回転した結果の向きを示しています．\n
  次に何かキーを押してみましょう．

  .. image:: ../../images/examples/tutorial_003_mug_rot_Zaxis.png
    :height: 200

- この次からは，キーを押す度に先ほどの黄色の矢印を軸にしてマグカップが回転しながら移動します．

解説
------------------------------------

.. code-block:: python

  from openravepy import Environment, rotationMatrixFromAxisAngle, axisAngleFromRotationMatrix, matrixFromAxisAngle
                                      
- openravepyから `Environment` , `rotationMatrixFromAxisAngle` , `axisAngleFromRotationMatrix` ,  `matrixFromAxisAngle` のモジュールを読み込んでいます．

.. code-block:: python

  from numpy import eye, dot, pi

- numpyから必要なモジュールを読み込んでいます．

.. code-block:: python

  handles=[]
  handles.append(env.drawarrow(p1=[0.0,0.0,0.0],p2=[0.5,0.0,0.0],linewidth=0.01,color=[1.0,0.0,0.0]))
  handles.append(env.drawarrow(p1=[0.0,0.0,0.0],p2=[0.0,0.5,0.0],linewidth=0.01,color=[0.0,1.0,0.0]))
  handles.append(env.drawarrow(p1=[0.0,0.0,0.0],p2=[0.0,0.0,0.5],linewidth=0.01,color=[0.0,0.0,0.1]))

- 矢印を描画するには `Environment.drawarrow` を使います．

.. code-block:: python

  deg = raw_input('X軸の回転角度を入力して下さい．[degree] X = ')

- X軸の回転角度のキー入力を待ち，degに代入します．

.. code-block:: python

  rot_mat = rotationMatrixFromAxisAngle([1,0,0],float(deg)*pi/180.0)

- `rotationMatrixFromAxisAngle` を使ってX軸で入力された角度（deg）回転する回転行列rot_matを作成しています．\n
  AxisAngleは例のように([単位ベクトル] , 角度(rad))でも指定することが可能です．

.. code-block:: python

  print 'AxisAngle = ',axisAngleFromRotationMatrix(rot_mat)

-  `axisAngleFromRotationMatrix` を使って実際のAxisAngleの値（3列のベクトル値）を出力します．

.. code-block:: python

  tran[0:3,0:3] = dot(rot_mat, tran[0:3,0:3])
  body.SetTransform(tran)

- 変換行列(tran)から現在の回転行列を切り出し，dot関数で回転行列(rot_mat)の掛け算をして，それを再び変換行列に代入して新たな姿勢をセットしています．

.. code-block:: python

  P1 = dot(rot_mat, [0,0,1])
  handles.append(env.drawarrow([0.0,0.0,0.0],P1,linewidth=0.01,color=[1.0,1.0,0.0]))

- カップの上方向（Z軸方向）を回転させて，黄色い矢印を表示させています．\n

.. code-block:: python

  deg = raw_input('Y軸の回転角度を入力して下さい．[degree] Y = ')
  rot_mat = rotationMatrixFromAxisAngle([0,1,0],float(deg)*pi/180.0)
  print 'AxisAngle = ',axisAngleFromRotationMatrix(rot_mat)
  tran[0:3,0:3] = dot(rot_mat, tran[0:3,0:3])
  body.SetTransform(tran)
  P2 = dot(rot_mat, P1)
  handles.append(env.drawarrow([0.0,0.0,0.0],P2,linewidth=0.01,color=[1.0,1.0,0.0]))

- Y軸の回転に関しても同様に行います．

.. code-block:: python

  while True:
      raw_input('キーを押すと回転しながら移動します．')
      Tdelta = matrixFromAxisAngle ([0,0,0.5])
      Tdelta[2,3] = 0.01
      tran = dot(tran, Tdelta)
      body.SetTransform(tran)

- Tdelta（Z軸の回転と移動）を作成し，dot関数で現在の変換行列tranとの掛け算をして，それを再びtranに代入して新たな姿勢をセットしています．\n
  これによりキーが入力される度に，カップの回転軸（黄色い矢印）で回転しながら移動します．\n

関連関数
--------------------------------------

- :meth:`.Environment.drawarrow` , :func:`.rotationMatrixFromAxisAngle` , :func:`.axisAngleFromRotationMatrix` , :func:`.matrixFromAxisAngle` , :func:`.matrixFromPose` , :func:`.matrixFromQuat` , :func:`.quatFromRotationMatrix` , :func:`.rotationMatrixFromQuat`

関連チュートリアル
--------------------------------------

- :mod:`.tutorial_004` - 環境に読み込んだ物体の回転（クォータニオン）

