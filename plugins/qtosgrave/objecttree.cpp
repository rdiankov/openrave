// -*- coding: utf-8 -*-
// Copyright (C) 2012 Gustavo Puche, Rosen Diankov, OpenGrasp Team
//
// OpenRAVE Qt/OpenSceneGraph Viewer is licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include "objecttree.h"

namespace qtosgrave {

ObjectTree::ObjectTree() {
    setFixedSize(100,200);
    model = OsgTreeModel::create();
}

ObjectTree::~ObjectTree() {
}

ObjectTree::OsgTreeModel *ObjectTree::OsgTreeModel::create()
{
    return new OsgTreeModel();
}

ObjectTree::OsgTreeModel::OsgTreeModel(){
    //set_column_types(_columns);
}

}
