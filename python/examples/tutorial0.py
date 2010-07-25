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
シミュレーション環境への物体の読み込み

ソースコード
--------------------------------------

.. code-block:: python

  #!/usr/bin/env python
  from openravepy import Environment
  from numpy import eye
  env = Environment()
  env.SetViewer('qtcoin')
  body = env.ReadKinBodyXMLFile(filename='data/mug1.kinbody.xml')
  env.AddKinBody(body)
  body.SetTransform(eye(4))

ソースコードの実行
--------------------------------------

まずは，OpenRAVEのインストールされたフォルダに移動しましょう．（以降は特別な指定がない限り，このフォルダで作業が行われていることとします．）

.. code-block:: bash

 cd `openrave-config --python-dir`

次にpythonを立ち上げます．（OpenRAVEでは，よりインタラクティブな機能を持ったipythonの使用を推奨しています．） 

.. code-block:: bash

 ipython

上のソースコードを入力（コピー＆ペースト）してください． 

すると，OpenRAVEのGUIが立ち上がり，下のような画像が現れます．

.. image:: ../../images/example_tutorials/001_mug_read_kinbody.png
  :height: 200

ソースコードの解説
--------------------------------------

それではソースコードを詳しく見てみましょう． 

.. code-block:: python

  from openravepy import Environment

これは，openravepyから，Enviromentのモジュールをインポートしています． 

.. code-block:: python

  from numpy import eye

これは，numpyから，eyeのモジュールをインポートしています．

.. code-block:: python

  env = Environment()

OpenRAVEシミュレーションの基本となる，Enviromentのインスタンスを作成しています．  

.. code-block:: python

 env.SetViewer('qtcoin')

ビューワとしてqtcoinを設定しています．

.. code-block:: python

  body = env.ReadKinBodyXMLFile(filename='data/mug1.kinbody.xml')

OpenRAVEがインストールされているフォルダ（$INSTALL/share/openrave）には，予めいくつかの物体，ロボット，環境のファイルが提供されています．今回はこの中のdataフォルダからmug1.kinbody.xmlをロードしています．

.. code-block:: python

  env.AddKinBody(body)

設定したオブジェクトをシミュレーション環境に加えています．この段階で初めてビューワに表示されます．

.. code-block:: python

  body.SetTransform(eye(4))

オブジェクトの位置姿勢をセットしています．今回は4x4の単位行列(np.eye(4))ですので，回転なし，移動なしとなります．
以上が基本的なオブジェクトを環境に加える方法です．


覚えておくべきこと
--------------------------------------

- 環境をリセットするには以下のコマンドを入力します．

.. code-block:: python

 env.Reset()

- OpenRAVEを安全に終了するために最後に必ず実行してください．

.. code-block:: python

  env.Destroy()
 
- 他の利用可能なビューワは端末から以下のコマンドで調べることができます．

.. code-block:: bash

 openrave.py --listviewer

また，逆運動学の解析などでビューワが必要ない場合，ビューワなしで実行することも可能です．その場合は，SetViewerを呼び出す必要はありません．


- pythonを起動しなくても，以下のコマンドで同じ事を端末から直接実行することも可能ですます．

.. code-block:: bash

  openrave.py data/mug1.kinbody.xml

- さらにこの後，物体の移動などを続けたい場合はそのままipythonにドロップインすることが可能です．その場合は-iオプションを指定して下さい． 

.. code-block:: bash

  openrave.py -i data/mug1.kinbody.xml

- 全てのOpenRAVEのexampleはopenrave.pyから直接実行する事が可能です．

.. code-block:: bash

  openrave.py -example tutorial0

- 他の実行可能なexampleは端末から以下のコマンドで調べることができます．

.. code-block:: bash

  openrave.py -listexample


覚えておくと便利なこと
--------------------------------------

- ipythonを終了するにはexit()またはquit()です．Ctrl+Dのショートカットも使用できます．


NEXT
--------------------------------------

環境に読み込んだ物体の移動

"""
from __future__ import with_statement # for python 2.5
__author__ = 'Makoto Furukawa'
__copyright__ = '2010 Makoto Furukawa'
__license__ = 'Apache License, Version 2.0'
from openravepy import Environment
from numpy import eye
from time import sleep
def run(args=None):
    env = Environment()
    env.SetViewer('qtcoin')
    body = env.ReadKinBodyXMLFile(filename='data/mug1.kinbody.xml')
    env.AddKinBody(body)
    body.SetTransform(eye(4))
    while True:
        sleep(0.01)

if __name__ == "__main__":
    run()
