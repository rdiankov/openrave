チュートリアル001：シミュレーション環境への物体の読み込み
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

 :作者: 古川誠

.. code-block:: python

  from openravepy import Environment, with_destroy
  from numpy import eye
  env = Environment()
  env.SetViewer('qtcoin')
  body = env.ReadKinBodyXMLFile(filename='data/mug1.kinbody.xml')
  env.AddKinBody(body)
  body.SetTransform(eye(4))


実行
--------------------------------------

実行方法
========

- openrave.pyから直接実行する方法

.. code-block:: bash

  openrave.py --example tutorial_001

- pythonからコマンドラインで実行する方法
  
  OpenRAVEでは，よりインタラクティブな機能を持ったipythonの使用を推奨しています．ipythonを立ち上げソースコードを順番に入力（コピー＆ペースト）して実行してください．

.. code-block:: bash

  ipython

内容説明
========

- チュートリアルを実行すると，OpenRAVEのGUIが立ち上がり，下のような画像が現れます．

  .. image:: ../../images/examples/tutorial_001_mug_read_kinbody.png
    :height: 200

解説
--------------------------------------

.. code-block:: python

  from openravepy import Environment

- openravepyから，Enviromentのモジュールをインポートしています． 

.. code-block:: python

  from numpy import eye

- numpyから，eyeのモジュールをインポートしています．

.. code-block:: python

  env = Environment()

- OpenRAVEシミュレーションの基本となる，Enviromentのインスタンスを作成しています．  

.. code-block:: python

 env.SetViewer('qtcoin')

- ビューワとしてqtcoinを設定しています．

.. code-block:: python

  body = env.ReadKinBodyXMLFile(filename='data/mug1.kinbody.xml')

- OpenRAVEがインストールされているフォルダ（$INSTALL/share/openrave）には，予めいくつかの物体，ロボット，環境のファイルが提供されています．\n
  今回はこの中のdataフォルダからmug1.kinbody.xmlをロードしています．

.. code-block:: python

  env.AddKinBody(body)

- 設定したオブジェクトをシミュレーション環境に加えています．この段階で初めてビューワに表示されます．

.. code-block:: python

  body.SetTransform(eye(4))

- オブジェクトの位置姿勢をセットしています．今回は4x4の単位行列(eye(4))ですので，回転・移動なしとなります．\n
  以上がオブジェクトを環境に加える基本的な方法です．

.. code-block:: python

  raw_input('キーを押すとリセットします．')
  env.Reset()

- 最後に、キー入力を待って環境をリセットします．

覚えておくべきこと
--------------------------------------

- pythonからコマンドラインで実行している場合は、OpenRAVEを安全に終了するために最後に次のコマンドを必ず実行してください．

.. code-block:: python

  env.Destroy()
 
- 他の利用可能なビューワは端末から以下のコマンドで調べることができます．\n
  また，逆運動学の解析などでビューワが必要ない場合，ビューワなしで実行することも可能です．その場合は，SetViewerを呼び出す必要はありません．

.. code-block:: bash

 openrave.py --listviewer

- pythonを起動しなくても，以下のコマンドで同じ事を端末から直接実行することも可能ですます．

.. code-block:: bash

  openrave.py data/mug1.kinbody.xml

- さらにこの後，物体の移動などを続けたい場合はそのままipythonにドロップインすることが可能です．その場合は-iオプションを指定して下さい． 

.. code-block:: bash

  openrave.py -i data/mug1.kinbody.xml

- 他の実行可能なexampleは端末から以下のコマンドで調べることができます．

.. code-block:: bash

  openrave.py --listexamples

覚えておくと便利なこと
--------------------------------------

- ipythonを終了するにはexit()またはquit()です．Ctrl+Dのショートカットも使用できます．

関連関数
--------------------------------------

- :class:`.Environment` , :meth:`.Environment.SetViewer` , :meth:`.Environment.ReadKinBodyXMLFile` , :meth:`.Environment.AddKinBody` , :meth:`.KinBody.SetTransform` , :meth:`.Environment.Reset` , :meth:`.Environment.Destroy`

関連チュートリアル
--------------------------------------

- :mod:`.tutorial_002` - 環境に読み込んだ物体の移動
