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
環境に読み込んだ物体の移動

ソースコード
--------------------------------------

.. code-block:: python

  #!/usr/bin/env python
  from openravepy import Environment
  from numpy import eye
  from numpy.random import rand
  env = Environment()
  env.SetViewer('qtcoin')
  body1 = env.ReadKinBodyXMLFile(filename='data/mug1.kinbody.xml')
  env.AddKinBody(body1)
  body1.SetTransform(eye(4))
  body2 = env.ReadKinBodyXMLFile(filename='data/mug2.kinbody.xml')
  env.AddKinBody(body2)
  body2.SetTransform(eye(4))
  raw_input('キーを押すと現在の変換行列を出力します．')
  tran1 = body1.GetTransform()
  print tran1
  raw_input('キーを押すとXに+0.5移動し，移動後の変換行列を出力します．')
  tran1[0,3] = 0.5 
  body1.SetTransform(tran1)
  print tran1
  while True:
      s  = raw_input("キーを押すとランダムに移動します．終了するには'q'を押してください．")
      if s is 'q':
          break
      else:
          tran1[0:3,3] = rand(1,3) 
          body1.SetTransform(tran1)
          print "X=%f Y=%f Z=%f"%(tran1[0,3],tran1[1,3],tran1[2,3])

ソースコードの実行
--------------------------------------

上のソースコードを実行してください． 

.. code-block:: bash

  openrave.py --example tutorial_002
  
すると，OpenRAVEのGUIが立ち上がり，下のような画像が現れます．

.. image:: ../../images/example_tutorials/002_two_mugs_read_kinbody.png
  :height: 200

いま同じ場所に２つのマグカップを読み出してしまったため，重なってしまいました．
このため，物体を移動させてみましょう．キーを１回押すと現在の変換行列を出力します．
もう一度キーを押すと物体が移動し，新しい変換行列を出力します．
すると，以下のような画像が現れるはずです．\n

.. image:: ../../images/example_tutorials/002_two_mugs_one_moved.png
  :height: 200

青色のマグカップがなくなってしまいました！
安心して下さい，青色のマグカップが無事に移動したからです．
この様な場合はマウスのスクロールボタンを使って手動でズームアウトするか，GUIの右側にある眼のアイコンを使ってビューを調整することができます．

.. image:: ../../images/example_tutorials/002_two_mugs_both_visible.png
  :height: 200

この次からは，キーを押す度に物体がランダムに移動します．
終了するには'q'を入力して下さい．


ソースコードの解説
--------------------------------------

それではソースコードを詳しく見てみましょう． 

.. code-block:: python

  from numpy.random import rand

これはnumpy.randomからrandのモジュールを読み込んでいます．後で物体の位置をランダムに生成するのに使用します．

.. code-block:: python

  body2 = env.ReadKinBodyXMLFile(filename='data/mug2.kinbody.xml')
  env.AddKinBody(body2)
  body2.SetTransform(eye(4))

2つ目の物体としてmug2を読み込んで環境に加えています．

.. code-block:: python

  raw_input('キーを押すと現在の変換行列を出力します．')

pythonの標準関数であるraw_inputはキーボードから何かの入力を待ちます．

.. code-block:: python

  tran1 = body1.GetTransform()
  print tran1

環境からGetTransform関数を使ってbody1（mug1）の現在の変換行列をtran1に読み出し，それをプリントで表示しています．

.. code-block:: python

  [[ 1.  0.  0.  0.]
   [ 0.  1.  0.  0.]
   [ 0.  0.  1.  0.]
   [ 0.  0.  0.  1.]]

OpenRaveでは物体の回転と移動を４×４の変換行列で行います．左上の３×３の行列が回転，右上の３×１の行列が移動を示しています．
body1はまだ回転も移動もしていませんので，プリントされた変換行列は上のような単位行列になっていることが確認できます．

.. code-block:: python

  raw_input('キーを押すとXに+0.5移動し，移動後の変換行列を出力します．')
  tran1[0,3] = 0.5 
  body1.SetTransform(tran1)
  print tran1

キーを押すと，X軸の移動を示す変換行列に+0.5を代入します．
その後，SetTransform関数で物体の変換行列がセットされ，環境に反映されて物体が移動します．

.. code-block:: python

  [[ 1.   0.   0.   0.5]
   [ 0.   1.   0.   0. ]
   [ 0.   0.   1.   0. ]
   [ 0.   0.   0.   1. ]]

X軸の移動は変換行列の０行３列目（tran1[0,3]）なので，そこに+0.5になっていることが確認できます．
その他，Y軸の移動は１行３列目（tran1[1,3]），Z軸の移動は２行３列目（tran1[2,3]）です．


.. code-block:: python

  while True:
    s  = raw_input("キーを押すとランダムに移動します．終了するには'q'を押してください．")
    if s is 'q':
      break
    else:
      tran1[0:3,3] = rand(1,3) 
      body1.SetTransform(tran1)
      print "X=%f Y=%f Z=%f"%(tran1[0,3],tran1[1,3],tran1[2,3])

最後はrand関数を使って物体の位置をランダムに作成し移動し，現在の位置を出力します．
whileループによって'q'が入力されるまで繰り返します．

NEXT
--------------------------------------

環境に読み込んだ物体の回転

"""
from __future__ import with_statement # for python 2.5
__author__ = 'Makoto Furukawa'
__copyright__ = '2010 Makoto Furukawa'
__license__ = 'Apache License, Version 2.0'
from openravepy import Environment
from numpy import eye
from numpy.random import rand

def run(args=None):
  env = Environment()
  env.SetViewer('qtcoin')
  body1 = env.ReadKinBodyXMLFile(filename='data/mug1.kinbody.xml')
  env.AddKinBody(body1)
  body1.SetTransform(eye(4))
  body2 = env.ReadKinBodyXMLFile(filename='data/mug2.kinbody.xml')
  env.AddKinBody(body2)
  body2.SetTransform(eye(4))
  raw_input('キーを押すと現在の変換行列を出力します．')
  tran1 = body1.GetTransform()
  print tran1
  raw_input('キーを押すとXに+0.5移動し，移動後の変換行列を出力します．')
  tran1[0,3] = 0.5 
  body1.SetTransform(tran1)
  print tran1
  while True:
    s  = raw_input("キーを押すとランダムに移動します．終了するには'q'を押してください．")
    if s is 'q':
      break
    else:
      tran1[0:3,3] = rand(1,3) 
      body1.SetTransform(tran1)
      print "X=%f Y=%f Z=%f"%(tran1[0,3],tran1[1,3],tran1[2,3])

if __name__ == "__main__":
    run()
