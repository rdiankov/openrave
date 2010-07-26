#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright (C) 2010 Makoto Furukawa
# 
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#     http://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""
環境に読み込んだ物体の回転（回転行列）

ソースコード
--------------------------------------

.. code-block:: python

  #!/usr/bin/env python
  from openravepy import Environment, rotationMatrixFromAxisAngle, axisAngleFromRotationMatrix
  from numpy import eye, dot, pi, array
  env = Environment()
  env.SetViewer('qtcoin')
  body1 = env.ReadKinBodyXMLFile(filename='data/mug2.kinbody.xml')
  env.AddKinBody(body1)
  body1.SetTransform(eye(4))
  tran1 = body1.GetTransform()
  handles=[]
  handles.append(env.drawarrow(p1=[0.0,0.0,0.0],p2=[0.5,0.0,0.0],linewidth=0.01,color=[1.0,0.0,0.0]))
  handles.append(env.drawarrow(p1=[0.0,0.0,0.0],p2=[0.0,0.5,0.0],linewidth=0.01,color=[0.0,1.0,0.0]))
  handles.append(env.drawarrow(p1=[0.0,0.0,0.0],p2=[0.0,0.0,0.5],linewidth=0.01,color=[0.0,0.0,0.1]))
  deg = raw_input("X軸の回転角度を入力して下さい．[degree] X = ")
  rot_mat = rotationMatrixFromAxisAngle([1,0,0],float(deg)*pi/180.0)
  tran1[0:3,0:3] = dot(rot_mat, tran1[0:3,0:3])
  body1.SetTransform(tran1)
  P1 = dot(rot_mat, [0,0,1])
  handles.append(env.drawarrow([0.0,0.0,0.0],P1,linewidth=0.01,color=[1.0,1.0,0.0]))
  deg = raw_input("Y軸の回転角度を入力して下さい．[degree] Y = ")
  rot_mat = rotationMatrixFromAxisAngle([0,1,0],float(deg)*pi/180.0)
  tran1[0:3,0:3] = dot(rot_mat, tran1[0:3,0:3])
  body1.SetTransform(tran1)
  P2 = dot(rot_mat, P1)
  handles.append(env.drawarrow([0.0,0.0,0.0],P2,linewidth=0.01,color=[1.0,1.0,0.0]))
  while True:
      raw_input("キーを押すと回転します．")
      tran1[0:3,0:3] = dot(tran1[0:3,0:3], rotationMatrixFromAxisAngle ([0,0,0.5]))
      body1.SetTransform(tran1)
  env.Destroy()
  print "The env was destroyed"

ソースコードの実行
--------------------------------------

上のソースコードを実行してください． 

.. code-block:: bash

  openrave.py --example tutorial_003

すると，OpenRAVEのGUIが立ち上がり，下のような画像が現れます．

.. image:: ../../images/example_tutorials/003_mug_origin.png
  :height: 200

画面内で矢印は座標軸を表し，RGBの色はそれぞれXYZ軸に対応しています．
X軸の回転角度を入力してみましょう．矢印の向きに向かって反時計まわりがプラスの回転方向です．

.. image:: ../../images/example_tutorials/003_mug_rot_Xaxis.png
  :height: 200

例ではX軸に-45度回転させました．新しく現れた黄色の矢印はカップの上方向（Z軸）が回転した結果の向きを示しています．\n
次にY軸の回転角度を入力してみましょう．

.. image:: ../../images/example_tutorials/003_mug_rot_Yaxis.png
  :height: 200

例ではY軸に+45度回転させました．黄色の矢印は先ほどと同じくカップの上方向（Z軸）が回転した結果の向きを示しています．\n
次に何かキーを押してみましょう．

.. image:: ../../images/example_tutorials/003_mug_rot_Zaxis.png
  :height: 200

この次からは，キーを押す度に先ほどの黄色の矢印を軸にして物体が回転します．

ソースコードの解説
--------------------------------------

それではソースコードを詳しく見てみましょう． 

.. code-block:: python

  from openravepy import Environment, rotationMatrixFromAxisAngle, axisAngleFromRotationMatrix

今回はopenravepyから`rotationMatrixFromAxisAngle`のモジュールを読み込んでいます．

.. code-block:: python

  from numpy import eye, dot, pi, array

次にnumpyからdot（行列の掛け算），pi（パイの値），array（配列の作成）のモジュールを読み込んでいます．

.. code-block:: python

  handles=[]
  handles.append(env.drawarrow(p1=[0.0,0.0,0.0],p2=[0.5,0.0,0.0],linewidth=0.01,color=[1.0,0.0,0.0]))
  handles.append(env.drawarrow(p1=[0.0,0.0,0.0],p2=[0.0,0.5,0.0],linewidth=0.01,color=[0.0,1.0,0.0]))
  handles.append(env.drawarrow(p1=[0.0,0.0,0.0],p2=[0.0,0.0,0.5],linewidth=0.01,color=[0.0,0.0,0.1]))

矢印を描画するにはEnvironment()クラスの関数`drawarrow`を使います．

.. code-block:: python

  deg = raw_input('X軸の回転角度を入力して下さい．[degree] X = ')
  rot_mat = rotationMatrixFromAxisAngle([1,0,0],float(deg)*pi/180.0)
  tran1[0:3,0:3] = dot(rot_mat, tran1[0:3,0:3])
  body1.SetTransform(tran1)

x軸を入力を待ち， `openravepy_int.rotationMatrixFromAxisAngle` を使ってX軸[1,0,0]で入力された角度回転する回転行列rot_matを作成しています．その後，現在の変換行列であるtran1から，回転行列の部分を（左上の3行3列）を切り出し，dot関数でrot_mat行列の掛け算をして，それをtran1に返して新たな姿勢をセットしています．

.. code-block:: python

  P1 = dot(rot_mat, [0,0,1])
  handles.append(env.drawarrow([0.0,0.0,0.0],P1,linewidth=0.01,color=[1.0,1.0,0.0]))

同様にカップの口の方向（Z軸）を回転させて，黄色い矢印を表示させています．\n
この後，Y軸の回転に関しても同じ事を行います．

.. code-block:: python

  while True:
      raw_input('キーを押すと回転します．')
      tran1[0:3,0:3] = dot(tran1[0:3,0:3], rotationMatrixFromAxisAngle ([0,0,0.5]))
      body1.SetTransform(tran1)

次にキーが入力されると，先の黄色い軸で回転します．初めに`rotationMatrixFromAxisAngle`でZ軸まわりの回転行列を求め，現在の回転行列を掛けて求めます．`rotationMatrixFromAxisAngle`の中で使われている[0,0,0.5]はAxis Angleと呼ばれ，これ自体で回転軸のベクトルと角度を表しています．

NEXT
--------------------------------------
環境に読み込んだ物体の回転（クォータニオン）


"""
from __future__ import with_statement # for python 2.5
__author__ = 'Makoto Furukawa'
__copyright__ = '2010 Makoto Furukawa'
__license__ = 'Apache License, Version 2.0'
from openravepy import Environment, rotationMatrixFromAxisAngle, axisAngleFromRotationMatrix, quatFromRotationMatrix
from numpy import eye, dot, pi, array
from numpy.random import rand
from time import time

def run(args=None):
    try:
        env = Environment()
        env.SetViewer('qtcoin')
        body1 = env.ReadKinBodyXMLFile(filename='data/mug2.kinbody.xml')
        env.AddKinBody(body1)
        body1.SetTransform(eye(4))
        tran1 = body1.GetTransform()
    
        handles=[]
        handles.append(env.drawarrow(p1=[0.0,0.0,0.0],p2=[0.5,0.0,0.0],linewidth=0.01,color=[1.0,0.0,0.0]))
        handles.append(env.drawarrow(p1=[0.0,0.0,0.0],p2=[0.0,0.5,0.0],linewidth=0.01,color=[0.0,1.0,0.0]))
        handles.append(env.drawarrow(p1=[0.0,0.0,0.0],p2=[0.0,0.0,0.5],linewidth=0.01,color=[0.0,0.0,0.1]))
    
        deg = raw_input('X軸の回転角度を入力して下さい．[degree] X = ')
        if len(deg) == 0:
            deg = -45
        rot_mat = rotationMatrixFromAxisAngle([1,0,0],float(deg)*pi/180.0)
        tran1[0:3,0:3] = dot(rot_mat, tran1[0:3,0:3])
        body1.SetTransform(tran1)

        P1 = dot(rot_mat, [0,0,1])
        handles.append(env.drawarrow([0.0,0.0,0.0],P1,linewidth=0.01,color=[1.0,1.0,0.0]))

        deg = raw_input('Y軸の回転角度を入力して下さい．[degree] Y = ')
        if len(deg) == 0:
            deg = 45
        rot_mat = rotationMatrixFromAxisAngle([0,1,0],float(deg)*pi/180.0)
        tran1[0:3,0:3] = dot(rot_mat, tran1[0:3,0:3])
        body1.SetTransform(tran1)

        P2 = dot(rot_mat, P1)
        handles.append(env.drawarrow([0.0,0.0,0.0],P2,linewidth=0.01,color=[1.0,1.0,0.0]))

        while True:
            raw_input('キーを押すと回転します．')
            tran1[0:3,0:3] = dot(tran1[0:3,0:3], rotationMatrixFromAxisAngle ([0,0,0.5]))
            body1.SetTransform(tran1)

    finally:
        env.Destroy()
        print 'The env was destroyed'

if __name__ == "__main__":
    run()
