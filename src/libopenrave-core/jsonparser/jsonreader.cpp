// -*- coding: utf-8 -*-
// Copyright (C) 2013 Rosen Diankov <rosen.diankov@gmail.com>
//
// This file is part of OpenRAVE.
// OpenRAVE is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
#include "jsoncommon.h"
#include <rapidjson/filereadstream.h>
#include <string>
namespace OpenRAVE {

	class JSONReader{

	public:

		JSONReader(EnvironmentBasePtr penv): _penv(penv), _prefix(""){}
		virtual ~JSONReader(){}

		std::string ReadFile(std::string fullFilename){
			std::ifstream fp;
		    fp.open(fullFilename.c_str(), std::ifstream::in);
		    std::string data;
		    data = std::string((std::istreambuf_iterator<char>(fp)), std::istreambuf_iterator<char>());
			return data;	
		}

		std::map<std::string, rapidjson::Value::ValueIterator> GetObjectMap(rapidjson::Document& d){
			std::map<std::string, rapidjson::Value::ValueIterator> retMap;
			
			if(d.HasMember("objects")){
				rapidjson::Value& objects = d["objects"];
			    for(rapidjson::Value::ValueIterator itr = objects.Begin(); itr!=objects.End();++itr){
			        std::string id = itr->FindMember("id")->value.GetString();
			        retMap[id] = itr;
			    }
			}
			return retMap;
		}

		rapidjson::Value::ValueIterator ResolveObject(std::string uri, std::map<std::string, rapidjson::Value::ValueIterator>& objectMap){
			// use # instead of @

			std::string scheme = GetScheme(uri);
			uri = uri.substr(uri.find(scheme) + scheme.size());
		    if(uri[0] == '#'){
		        // in the same file
		        // remove the first @ char
		        std::string id = uri.substr(1);
		        return objectMap[id];
		    }
		    else{
		    	int splitPos = uri.find('#');
		    	std:;string data = "";
		    	if(scheme == "file:"){
		    		std::string otherFilename = RaveFindLocalFile( uri.substr(0, splitPos-1)  );
			    	data = ReadFile(otherFilename);
		    	}
		    	else if(find(_vOpenRAVESchemeAliases.begin(), _vOpenRAVESchemeAliases.end(), scheme) != _vOpenRAVESchemeAliases.end() ){
		    		// TODO deal with openrave: or mujin:
		    		
		    	}
		    	
		    	if(data.size() == 0){
		    		return rapidjson::Value::ValueIterator();
		    	}
		    	else{

		    		rapidjson::StringStream s(data.c_str());
					rapidjson::Document d;
					d.SetObject();
					d.ParseStream(s);
		    		std::map<std::string, rapidjson::Value::ValueIterator> otherObjectMap = GetObjectMap(d);
		    		std::string id = uri.substr(splitPos+1);
		    		return otherObjectMap[id];
		    	}
		    }
		}

		void AddFullURI(rapidjson::Document& d, rapidjson::Value::ValueIterator& body, std::string filename){

			if( body->HasMember("uri") ){
				std::string uri = body->FindMember("uri")->value.GetString();

				if(uri.size() == 0 ){
					return;
				}

				if(_prefix.size() > 0){
					uri = _prefix + uri;
				}

				else if(uri[0] == '#'){
					uri = filename + uri;
				}

				if(body->RemoveMember("uri")){
					rapidjson::Value key;
					rapidjson::Value value(uri, d.GetAllocator());
					key.SetString("uri", d.GetAllocator());
					body->AddMember(key, value, d.GetAllocator());
				}
			}
		}

		bool ApplyBodies(rapidjson::Document& d, std::string filename){
			if(!d.HasMember("bodies")){
		    	return false;
		    }
		   	std::map<std::string, rapidjson::Value::ValueIterator> objectMap = GetObjectMap(d);

		    rapidjson::Value& bodies = d["bodies"];
		    for(rapidjson::Value::ValueIterator itr=bodies.Begin(); itr!=bodies.End(); ++itr)
		    {
		        if(itr->HasMember("uri")){
		            std::string uri = itr->FindMember("uri")->value.GetString();
		            rapidjson::Value::ValueIterator object = ResolveObject(uri, objectMap);

		            // add the object items into instobjects
		            for(rapidjson::Value::MemberIterator memitr = object->MemberBegin(); memitr !=object->MemberEnd();++memitr){
		                std::string keyname = memitr->name.GetString();
		                rapidjson::Value key;
		                key.SetString(keyname.c_str(), d.GetAllocator());
		                rapidjson::Value value( memitr->value, d.GetAllocator() );
		                itr->AddMember(key, value, d.GetAllocator());
		            }
		            AddFullURI(d, itr, filename);
		            // set full uri to the uri path
		        }
		    }
		    return true;
		}


		bool _InitPreOpen(const AttributesList& atts){

			FOREACHC(itatt, atts){
				if( itatt->first == "prefix"){
					_prefix = itatt->second;
				}
				else if(itatt->first == "openravescheme")
				{
					 std::stringstream ss(itatt->second);
	                _vOpenRAVESchemeAliases = std::vector<std::string>((istream_iterator<std::string>(ss)), istream_iterator<std::string>());
				}
			}
			if(_vOpenRAVESchemeAliases.size() == 0){
				_vOpenRAVESchemeAliases.push_back("openrave");
			}
			return true;
		}

		bool _InitPostOpen(const AttributesList& atts){
			return true;
		}

		std::string GetScheme(std::string uriorigin){
			return uriorigin.substr(0, uriorigin.find(':')+1);
		}

		bool InitFromFile(const string& filename, const AttributesList& atts){
			_InitPreOpen(atts);

			_filename = filename;

			return _InitPostOpen(atts);
		}

		bool Extract(){
			std::string data = ReadFile(_filename);
			if(data.size() == 0)
			{
				return false;
			}
			rapidjson::StringStream s(data.c_str());
			rapidjson::Document d;
			d.SetObject();
			d.ParseStream(s);

		    bool success = ApplyBodies(d, _filename);

		    if(!success){
		    	return false;
		    }
		    _penv->DeserializeJSON(d);
			return true;
		}

	private:
		std::string _prefix;
		std::string _filename;
		EnvironmentBasePtr _penv;
		std::vector<std::string> _vOpenRAVESchemeAliases;
	};
	bool RaveParseJSONFile(EnvironmentBasePtr penv, const std::string& filename,const AttributesList& atts){

		std::string fullFilename = RaveFindLocalFile(filename);

		JSONReader reader(penv);
		if(fullFilename.size() == 0 || ! reader.InitFromFile(fullFilename, atts)){
			return false;
		}
		return reader.Extract();
	}

	// bool RaveParseJSONFile(EnvironmentBasePtr penv, KinBodyPtr& ppbody, const std::string& filename,const AttributesList& atts){

	// 	std::string fullFilename = RaveFindLocalFile(filename);

	// 	std::string data = ReadFile(fullFilename);

	// 	if(data.size() == 0){
	// 		return false;
	// 	}
	// 	rapidjson::StringStream s(data.c_str());
	// 	rapidjson::Document d;
	// 	d.SetObject();
	// 	d.ParseStream(s);

	// 	bool success =ApplyBodies(d, filename);

	// 	if(! success){
	// 		return false;
	// 	}
	// 	ppbody->DeserializeJSON(d);
	// 	return true;
	// }
	// bool RaveParseJSONFile(EnvironmentBasePtr penv, RobotBasePtr& pprobot, const std::string& filename, const AttributesList& atts){

	// 	std::string fullFilename = RaveFindLocalFile(filename);

	// 	std::string data = ReadFile(fullFilename);

	// 	if(data.size() == 0){
	// 		return false;
	// 	}
	// 	rapidjson::StringStream s(data.c_str());
	// 	rapidjson::Document d;
	// 	d.SetObject();
	// 	d.ParseStream(s);

	// 	bool success =ApplyBodies(d, filename);

	// 	if(! success){
	// 		return false;
	// 	}

	// 	pprobot->DeserializeJSON(d);
	// 	return true;
	// }
}
