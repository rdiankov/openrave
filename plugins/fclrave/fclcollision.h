#ifndef OPENRAVE_FCL_COLLISION
#define OPENRAVE_FCL_COLLISION

#include <boost/unordered_set.hpp>
#include <boost/lexical_cast.hpp>
#include <openrave/utils.h>

#include "fclspace.h"

typedef FCLSpace::KinBodyInfoConstPtr KinBodyInfoConstPtr;
typedef FCLSpace::KinBodyInfoPtr KinBodyInfoPtr;

class FCLCollisionChecker : public OpenRAVE::CollisionCheckerBase
{
public:


    struct CollisionCallbackData {
      CollisionCallbackData(boost::shared_ptr<FCLCollisionChecker> pchecker) : _pchecker(pchecker), _bCollision(false)
        {
        }

      // Do I still need this penv ?
      boost::shared_ptr<FCLCollisionChecker> _pchecker;
        fcl::CollisionRequest _request;
        fcl::CollisionResult _result;
        CollisionReportPtr _report;
      std::list<EnvironmentBase::CollisionCallbackFn> listcallbacks;

      // TODO : not used/filled anywhere...
        boost::unordered_set<LinkPair> disabledPairs;
        boost::unordered_set<LinkPair> selfEnabledPairs;

        bool _bCollision;
    };

    FCLCollisionChecker(OpenRAVE::EnvironmentBasePtr penv)
        : OpenRAVE::CollisionCheckerBase(penv)
    {
        _userdatakey = std::string("fclcollision") + boost::lexical_cast<std::string>(this);
        _fclspace.reset(new FCLSpace(penv, _userdatakey));
        _options = 0;
        _numMaxContacts = std::numeric_limits<int>::max(); // TODO 
        __description = ":Interface Author:\n\nFlexible Collision Library collision checker";

        RegisterCommand("SetBroadphaseAlgorithm", boost::bind(&FCLCollisionChecker::_SetBroadphaseAlgorithm, this, _1, _2), "sets the broadphase algorithm (Naive, SaP, SSaP, IntervalTree, DynamicAABBTree, DynamicAABBTree_Array)");

        RegisterCommand("SetBVHRepresentation", boost::bind(&FCLCollisionChecker::_SetBVHRepresentation, this, _1, _2), "sets the Bouding Volume Hierarchy representation for meshes (AABB, OBB, OBBRSS, RSS, kIDS)");
    }

    virtual ~FCLCollisionChecker() {
        DestroyEnvironment();
    }

    // TODO : What about Clone ?

    virtual bool SetCollisionOptions(int collision_options)
    {
      _options = collision_options;

      // TODO : remove when distance is implemented
      if( _options & OpenRAVE::CO_Distance ) {
        return false;
      }
        return true;
    }

    virtual int GetCollisionOptions() const
    {
        return _options;
    }

    virtual void SetTolerance(OpenRAVE::dReal tolerance)
    {
    }

    bool _SetBroadphaseAlgorithm(ostream& sout, istream& sinput)
    {
        std::string algorithm;
        sinput >> algorithm;
        _fclspace->SetBroadphaseAlgorithm(algorithm);
        return !!sinput;
    }

    bool _SetBVHRepresentation(ostream& sout, istream& sinput)
    {
        std::string type;
        sinput >> type;
        _fclspace->SetBVHRepresentation(type);
        return !!sinput;
    }


    virtual bool InitEnvironment()
    {
        vector<KinBodyPtr> vbodies;
        GetEnv()->GetBodies(vbodies);
        FOREACHC(itbody, vbodies) {
            InitKinBody(*itbody);
        }
        return true;
    }

    virtual void DestroyEnvironment()
    {
        _fclspace->DestroyEnvironment();
    }

    virtual bool InitKinBody(OpenRAVE::KinBodyPtr pbody)
    {
        FCLSpace::KinBodyInfoPtr pinfo = boost::dynamic_pointer_cast<FCLSpace::KinBodyInfo>(pbody->GetUserData(_userdatakey));
        if( !pinfo || pinfo->GetBody() != pbody ) {
            pinfo = _fclspace->InitKinBody(pbody);
        }
        return !pinfo;
    }

    virtual void RemoveKinBody(OpenRAVE::KinBodyPtr pbody)
    {
        _fclspace->RemoveUserData(pbody);
    }

    virtual bool CheckCollision(KinBodyConstPtr pbody1, CollisionReportPtr report = CollisionReportPtr())
    {
        static std::vector<KinBodyConstPtr> const vbodyexcluded;
        static std::vector<LinkConstPtr> const vlinkexcluded;

        return CheckCollision(pbody1, vbodyexcluded, vlinkexcluded, report);
    }

    virtual bool CheckCollision(KinBodyConstPtr pbody1, KinBodyConstPtr pbody2, CollisionReportPtr report = CollisionReportPtr())
    {
      if( pbody1->GetLinks().size() == 0 || !pbody1->IsEnabled() ) {
        return false;
      }

      if( pbody2->GetLinks().size() == 0 || !pbody2->IsEnabled() ) {
        return false;
      }

      if( pbody1->IsAttached(pbody2) ) {
        return false;
      }

      // Do we really want to synchronize everything ?
      _fclspace->Synchronize();


      CollisionGroup body1group, body2group;
      Collect(pbody1, body1group, _options & OpenRAVE::CO_ActiveDOFs); // This is asymetric : we consider everything attached to an active link of the first body and anything attached to the second
      Collect(pbody2, body2group, false);

      BroadPhaseCollisionManagerPtr body1manager = _fclspace->CreateManager(), body2manager = _fclspace->CreateManager();
      body1manager->registerObjects(body1group);
      body2manager->registerObjects(body2group);

      body1manager->setup();
      body2manager->setup();

      if( _options & OpenRAVE::CO_Distance ) {
        RAVELOG_WARN("fcl doesn't support CO_Distance yet\n");
        return false; //TODO
      } else {
        boost::shared_ptr<CollisionCallbackData> pquery = SetupCollisionQuery(report);
        body1manager->collide(body2manager.get(), pquery.get(), &FCLCollisionChecker::NarrowPhaseCheckCollision);
        return pquery->_bCollision;
      }

    }

    virtual bool CheckCollision(LinkConstPtr plink,CollisionReportPtr report = CollisionReportPtr())
    {
        static std::vector<KinBodyConstPtr> const vbodyexcluded;
        static std::vector<LinkConstPtr> const vlinkexcluded;

        return CheckCollision(plink, vbodyexcluded, vlinkexcluded, report);
    }

    virtual bool CheckCollision(LinkConstPtr plink1, LinkConstPtr plink2, CollisionReportPtr report = CollisionReportPtr())
    {
      // why do we synchronize everything ?
      _fclspace->Synchronize();

      CollisionGroup link1Group, link2Group;
      Collect(plink1, link1Group);
      Collect(plink2, link2Group);

      BroadPhaseCollisionManagerPtr link1Manager = _fclspace->CreateManager(), link2Manager = _fclspace->CreateManager();
      link1Manager->registerObjects(link1Group);
      link2Manager->registerObjects(link2Group);

      link1Manager->setup();
      link2Manager->setup();

      if( _options & OpenRAVE::CO_Distance ) {
        RAVELOG_WARN("fcl doesn't support CO_Distance yet\n");
        return false; // TODO
      } else {
        boost::shared_ptr<CollisionCallbackData> pquery = SetupCollisionQuery(report);
        link1Manager->collide(link2Manager.get(), pquery.get(), &FCLCollisionChecker::NarrowPhaseCheckCollision);
        return pquery->_bCollision;
      }
    }

    virtual bool CheckCollision(LinkConstPtr plink, KinBodyConstPtr pbody,CollisionReportPtr report = CollisionReportPtr())
    {
      // test done another time when collecting, but well...
      if( !plink->IsEnabled() ) {
        return false;
      }

      if( pbody->GetLinks().size() == 0 || !pbody->IsEnabled() ) {
        return false;
      }

      if( pbody->IsAttached(plink->GetParent()) ) {
        return false;
      }


      // Do we really need to synchronize everything ?
      _fclspace->Synchronize();

      CollisionGroup linkGroup, bodyGroup;
      Collect(plink, linkGroup);
      Collect(pbody, bodyGroup, false); // seems that activeDOFs are not considered in oderave : the check is done but it will always return true

      BroadPhaseCollisionManagerPtr linkManager = _fclspace->CreateManager(), bodyManager = _fclspace->CreateManager();
      linkManager->registerObjects(linkGroup);
      bodyManager->registerObjects(bodyGroup);

      linkManager->setup();
      bodyManager->setup();

      if( _options & OpenRAVE::CO_Distance ) {
        RAVELOG_WARN("fcl doesn't support CO_Distance yet\n");
        return false; // TODO
      } else {
        boost::shared_ptr<CollisionCallbackData> pquery = SetupCollisionQuery(report);
        linkManager->collide(bodyManager.get(), pquery.get(), &FCLCollisionChecker::NarrowPhaseCheckCollision);
        return pquery->_bCollision;
      }
    }

    virtual bool CheckCollision(LinkConstPtr plink, std::vector<KinBodyConstPtr> const &vbodyexcluded, std::vector<LinkConstPtr> const &vlinkexcluded, CollisionReportPtr report = CollisionReportPtr())
    {

      if( !plink->IsEnabled() ) {
        return false;
      }

        _fclspace->Synchronize();

        CollisionGroup linkCollisionGroup, exclusionGroup;
        Collect(plink, linkCollisionGroup, vlinkexcluded);

        FOREACH(itbody, vbodyexcluded) {
          Collect(*itbody, exclusionGroup, false);
        }
        FOREACH(itlink, vlinkexcluded) {
          Collect(*itlink, exclusionGroup);
        }

        BroadPhaseCollisionManagerPtr linkManager = _fclspace->CreateManager();
        linkManager->registerObjects(linkCollisionGroup);
        // TODO : check that there is no other link to take into consideration

        BroadPhaseCollisionManagerPtr envManager = _fclspace->GetEnvManager();
        UnregisterObjects(envManager, linkCollisionGroup);
        UnregisterObjects(envManager, exclusionGroup);

        linkManager->setup();
        envManager->setup();

        bool result;

        if( _options & OpenRAVE::CO_Distance ) {
          result = false;
        } else {
          boost::shared_ptr<CollisionCallbackData> pquery = SetupCollisionQuery(report);
          linkManager->collide(envManager.get(), pquery.get(), &FCLCollisionChecker::NarrowPhaseCheckCollision);
          result = pquery->_bCollision;
        }

        envManager->registerObjects(linkCollisionGroup);
        envManager->registerObjects(exclusionGroup);
        return result;
    }

    virtual bool CheckCollision(KinBodyConstPtr pbody, std::vector<KinBodyConstPtr> const &vbodyexcluded, std::vector<LinkConstPtr> const &vlinkexcluded, CollisionReportPtr report = CollisionReportPtr())
    {
      if( (pbody->GetLinks().size() == 0) || !pbody->IsEnabled() ) {
        return false;
      }

      _fclspace->Synchronize();

      // totally a bad idea... should really not do this kind of stuff
      boost::shared_ptr<CollisionGroup> pexclusionGroup = boost::make_shared<CollisionGroup>();
      CollisionGroup bodyGroup, &exclusionGroup = *pexclusionGroup;
      Collect(pbody, bodyGroup, !!(_options & OpenRAVE::CO_ActiveDOFs), vbodyexcluded, vlinkexcluded, pexclusionGroup);

      FOREACH(itbody, vbodyexcluded) {
        Collect(*itbody, exclusionGroup, false);
      }
      FOREACH(itlink, vlinkexcluded) {
        Collect(*itlink, exclusionGroup);
      }

      BroadPhaseCollisionManagerPtr bodyManager = _fclspace->CreateManager();
      bodyManager->registerObjects(bodyGroup);

      BroadPhaseCollisionManagerPtr envManager = _fclspace->GetEnvManager();
      UnregisterObjects(envManager, bodyGroup);
      UnregisterObjects(envManager, exclusionGroup);

      bodyManager->setup();
      envManager->setup();

      bool result;
      if( _options & OpenRAVE::CO_Distance ) {
        RAVELOG_WARN("fcl doesn't support CO_Distance yet\n");
        result = false; // TODO
      } else {
        boost::shared_ptr<CollisionCallbackData> pquery = SetupCollisionQuery(report);
        //BOOST_ASSERT(testValidity(bodyManager));
        //BOOST_ASSERT(testValidity(envManager));

        //TODO : test the validity of the two managers
        bodyManager->collide(envManager.get(), pquery.get(), &FCLCollisionChecker::NarrowPhaseCheckCollision);
        result = pquery->_bCollision;
      }

      envManager->registerObjects(bodyGroup);
      envManager->registerObjects(exclusionGroup);

      return result;
    }

    virtual bool CheckCollision(RAY const &ray, LinkConstPtr plink,CollisionReportPtr report = CollisionReportPtr())
    {
      RAVELOG_WARN("fcl doesn't support Ray collisions\n");
        return false; //TODO
    }

    virtual bool CheckCollision(RAY const &ray, KinBodyConstPtr pbody, CollisionReportPtr report = CollisionReportPtr())
    {
      RAVELOG_WARN("fcl doesn't support Ray collisions\n");
        return false; //TODO
    }

    virtual bool CheckCollision(RAY const &ray, CollisionReportPtr report = CollisionReportPtr())
    {
      RAVELOG_WARN("fcl doesn't support Ray collisions\n");
        return false; //TODO
    }

    virtual bool CheckStandaloneSelfCollision(KinBodyConstPtr pbody, CollisionReportPtr report = CollisionReportPtr())
    {
      if( pbody->GetLinks().size() <= 1 ) {
        return false;
      }

      int adjacentOptions = KinBody::AO_Enabled;
      if( (_options & OpenRAVE::CO_ActiveDOFs) && pbody->IsRobot() ) {
        adjacentOptions |= KinBody::AO_ActiveDOFs;
      }

      const std::set<int> &nonadjacent = pbody->GetNonAdjacentLinks(adjacentOptions);
      _fclspace->Synchronize();

      std::vector<BroadPhaseCollisionManagerPtr> mlinkManagers(pbody->GetLinks().size());

      if( _options & OpenRAVE::CO_Distance ) {
        RAVELOG_WARN("fcl doesn't support CO_Distance yet\n");
        return false; // TODO
      } else {
        boost::shared_ptr<CollisionCallbackData> pquery = SetupCollisionQuery(report);
        FOREACH(itset, nonadjacent) {
          size_t index1 = *itset&0xffff, index2 = *itset>>16;
          LinkConstPtr plink1(pbody->GetLinks().at(index1)), plink2(pbody->GetLinks().at(index2));
          BOOST_ASSERT( plink1->IsEnabled() && plink2->IsEnabled() );
          if( !mlinkManagers.at(index1) ) {
            mlinkManagers[index1] = _fclspace->CreateManager();
            CollisionGroup link1Group;
            Collect(plink1, link1Group);
            mlinkManagers[index1]->registerObjects(link1Group);
            mlinkManagers[index1]->setup();
          }
          if( !mlinkManagers.at(index2) ) {
            mlinkManagers[index2] = _fclspace->CreateManager();
            CollisionGroup link2Group;
            Collect(plink2, link2Group);
            mlinkManagers[index2]->registerObjects(link2Group);
            mlinkManagers[index2]->setup();
          }
          mlinkManagers[index1]->collide(mlinkManagers[index2].get(), pquery.get(), &FCLCollisionChecker::NarrowPhaseCheckCollision);

          if( pquery->_bCollision ) {
            if( IS_DEBUGLEVEL(OpenRAVE::Level_Verbose) ) {
              RAVELOG_VERBOSE(str(boost::format("selfcol %s, Links %s %s are colliding\n")%pbody->GetName()%plink1->GetName()%plink2->GetName()));
              std::vector<OpenRAVE::dReal> v;
              pbody->GetDOFValues(v);
              stringstream ss; ss << std::setprecision(std::numeric_limits<OpenRAVE::dReal>::digits10+1);
              for(size_t i = 0; i < v.size(); ++i ) {
                if( i > 0 ) {
                  ss << "," << v[i];
                }
                else {
                  ss << "colvalues=[" << v[i];
                }
              }
              ss << "]";
              RAVELOG_VERBOSE(ss.str());
            }
            if( !(_options & OpenRAVE::CO_AllLinkCollisions) ) {
              return true;
            }
          }
        }
        return pquery->_bCollision;
      }
    }

    virtual bool CheckStandaloneSelfCollision(LinkConstPtr plink, CollisionReportPtr report = CollisionReportPtr())
    {
      KinBodyPtr pbody = plink->GetParent();
      if( pbody->GetLinks().size() <= 1 ) {
        return false;
      }

      int adjacentOptions = KinBody::AO_Enabled;
      if( (_options & OpenRAVE::CO_ActiveDOFs) && pbody->IsRobot() ) {
        adjacentOptions |= KinBody::AO_ActiveDOFs;
      }

      const std::set<int> &nonadjacent = pbody->GetNonAdjacentLinks(adjacentOptions);

      _fclspace->Synchronize();

      CollisionGroup linkGroup, bodyGroup;

      Collect(plink, linkGroup);

      FOREACH(itset, nonadjacent) {
        KinBody::LinkConstPtr plink1(pbody->GetLinks().at(*itset&0xffff)), plink2(pbody->GetLinks().at(*itset>>16));
        if( plink == plink1 ) {
          Collect(plink2, bodyGroup);
        } else if( plink == plink2 ) {
          Collect(plink1, bodyGroup);
        }
      }

      BroadPhaseCollisionManagerPtr linkManager = _fclspace->CreateManager(), bodyManager = _fclspace->CreateManager();
      linkManager->registerObjects(linkGroup);
      bodyManager->registerObjects(bodyGroup);

      linkManager->setup();
      bodyManager->setup();

      if( _options & OpenRAVE::CO_Distance ) {
        RAVELOG_WARN("fcl doesn't support CO_Distance yet\n");
        return false; //TODO
      } else {
        boost::shared_ptr<CollisionCallbackData> pquery = SetupCollisionQuery(report);
        linkManager->collide(bodyManager.get(), pquery.get(), &NarrowPhaseCheckCollision);
        return pquery->_bCollision;
      }
    }

    void SetGeometryGroup(const std::string& groupname)
    {
        _fclspace->SetGeometryGroup(groupname);
    }

    const std::string& GetGeometryGroup() const
    {
        return _fclspace->GetGeometryGroup();
    }



private:
  inline boost::shared_ptr<FCLCollisionChecker> shared_checker() {
    return boost::dynamic_pointer_cast<FCLCollisionChecker>(shared_from_this());
  }

    void UnregisterObjects(BroadPhaseCollisionManagerPtr manager, CollisionGroup& group)
    {
        FOREACH(itcollobj, group) {
            manager->unregisterObject(*itcollobj);
        }
    }

    boost::shared_ptr<CollisionCallbackData> SetupCollisionQuery(CollisionReportPtr report)
    {
      boost::shared_ptr<CollisionCallbackData> pcb = boost::make_shared<CollisionCallbackData>(shared_checker());
      pcb->_report = report;

      if( GetEnv()->HasRegisteredCollisionCallbacks() && !report ) {
        pcb->_report.reset(new CollisionReport());
      }

      if( !!report && !!(report->options & OpenRAVE::CO_Contacts)) {
        pcb->_request.num_max_contacts = _numMaxContacts;
        pcb->_request.enable_contact = true;
      }

      if( !!pcb->_report ) {
        pcb->_report->Reset(_options);
      }

      return pcb;
    }


    boost::shared_ptr<CollisionCallbackData> SetupDistanceQuery(CollisionReportPtr report)
    {
        // TODO
      return boost::make_shared<CollisionCallbackData>(shared_checker());
    }

    void Collect(KinBodyConstPtr pbody, CollisionGroup &group, bool bactiveDOFs, std::vector<KinBodyConstPtr> const &vbodyexcluded = std::vector<KinBodyConstPtr>(), std::vector<LinkConstPtr> const &vlinkexcluded = std::vector<LinkConstPtr>(), boost::shared_ptr<CollisionGroup> pexclusionGroup = boost::shared_ptr<CollisionGroup>())
    {

      bool bActiveLinksOnly = false;
      if( bactiveDOFs && pbody->IsRobot() ) {

        RobotBaseConstPtr probot = OpenRAVE::RaveInterfaceConstCast<RobotBase>(pbody);
        bActiveLinksOnly = !probot->GetAffineDOF();

        if( bActiveLinksOnly ) {
          std::vector<bool> vactiveLinks = std::vector<bool>(probot->GetLinks().size(), false);
          for(size_t i = 0; i < probot->GetLinks().size() ; ++i) {
            bool isLinkActive = false;
            FOREACH(itindex, probot->GetActiveDOFIndices()) {
              isLinkActive = isLinkActive || probot->DoesAffect(probot->GetJointFromDOFIndex(*itindex)->GetJointIndex(), i);
            }
            if( isLinkActive ) {
              Collect(pbody->GetLinks()[i], group, vlinkexcluded);
            } else if ( !!pexclusionGroup ) {
              Collect(pbody->GetLinks()[i], *pexclusionGroup, vlinkexcluded);
            }
            vactiveLinks[i] = true;
          }

          std::set<KinBodyPtr> attachedBodies;
          pbody->GetAttached(attachedBodies);

          FOREACH(itbody, attachedBodies) {
            // TODO : should have a better solution
            if(*itbody != pbody || std::find(vbodyexcluded.begin(), vbodyexcluded.end(), *itbody) == vbodyexcluded.end()) {
              KinBody::LinkPtr pgrabbinglink = probot->IsGrabbing(*itbody);
              if( !!pgrabbinglink && vactiveLinks[pgrabbinglink->GetIndex()]) {
                FOREACH(itlink, (*itbody)->GetLinks()) {
                  Collect(*itlink, group, vlinkexcluded);
                }
              }
            }
          }
        }
      }

      if( !bActiveLinksOnly ) {
        std::set<KinBodyPtr> attachedBodies;
        pbody->GetAttached(attachedBodies);

        FOREACH(itbody, attachedBodies) {
          if(std::find(vbodyexcluded.begin(), vbodyexcluded.end(), *itbody) == vbodyexcluded.end()) {
            FOREACH(itlink, (*itbody)->GetLinks()) {
              Collect(*itlink, group, vlinkexcluded);
            }
          }
        }
      }
    }

    // Consider changing vlinkexcluded to something more suited to membership requests
    void Collect(LinkConstPtr plink, CollisionGroup &group, std::vector<LinkConstPtr> const &vlinkexcluded = std::vector<LinkConstPtr>())
  {
    // TODO : should not take into account the non-active links when CO_ActiveDOFs is set...
    if( !plink->IsEnabled() || std::find(vlinkexcluded.begin(), vlinkexcluded.end(), plink) != vlinkexcluded.end()) {
      return;
    }

    _fclspace->GetCollisionObjects(plink, group);
  }



  // Should-I follow the example of oderave and make this a method ?
  static bool NarrowPhaseCheckCollision(fcl::CollisionObject *o1, fcl::CollisionObject *o2, void *data) {
    CollisionCallbackData* pcb = static_cast<CollisionCallbackData *>(data);
    return pcb->_pchecker->NarrowPhaseCheckCollision(o1, o2, pcb);
  }

    bool NarrowPhaseCheckCollision(fcl::CollisionObject *o1, fcl::CollisionObject *o2, CollisionCallbackData* pcb)
    {
      static CollisionReport tmpReport;
      bool bHasCallbacks = GetEnv()->HasRegisteredCollisionCallbacks();
        LinkConstPtr plink1 = GetCollisionLink(*o1), plink2 = GetCollisionLink(*o2);

        if( !plink1 ) {
          RAVELOG_WARN("link1 still present in fcl space but no more in openrave");
          return false; //TODO
        }

        if( !plink2 ) {
          RAVELOG_WARN("link2 still present in fcl space but no more in openrave");
          return false; //TODO
        }

        LinkPair linkPair = MakeLinkPair(plink1, plink2);

        // Proceed to the next if the pair is disable
        if(pcb->disabledPairs.count(linkPair)) {
            return false;
        }

        // Proceed to the next if the links are attached and not enabled
        if( plink1->GetParent()->IsAttached(KinBodyConstPtr(plink2->GetParent()))
            && !pcb->selfEnabledPairs.count(linkPair) ) {
            return false;
        }

        size_t numContacts = fcl::collide(o1, o2, pcb->_request, pcb->_result);

        if( numContacts > 0 ) {

          if( !!pcb->_report ) {
            tmpReport.Reset(_options);
            tmpReport.plink1 = plink1;
            tmpReport.plink2 = plink2;

              if( _options & OpenRAVE::CO_Contacts ) {
                tmpReport.contacts.resize(numContacts);
                for(size_t i = 0; i < numContacts; ++i) {
                  fcl::Contact const &c = pcb->_result.getContact(i);
                  tmpReport.contacts[i] = CollisionReport::CONTACT(ConvertVectorFromFCL(c.pos), ConvertVectorFromFCL(c.normal), c.penetration_depth);
                }
              }

              if( bHasCallbacks ) {

                if(pcb->listcallbacks.size() == 0) {
                  GetEnv()->GetRegisteredCollisionCallbacks(pcb->listcallbacks);
                }

                OpenRAVE::CollisionAction action = OpenRAVE::CA_DefaultAction;
                CollisionReportPtr preport(&tmpReport, OpenRAVE::utils::null_deleter());
                FOREACH(callback, pcb->listcallbacks) {
                  action = (*callback)(preport, false);
                  if( action == OpenRAVE::CA_Ignore ) {
                    return false;
                  }
                }
              }

              pcb->_report->plink1 = tmpReport.plink1;
              pcb->_report->plink2 = tmpReport.plink2;
              if(pcb->_report->contacts.size() + numContacts < pcb->_report->contacts.capacity()) {
                pcb->_report->contacts.reserve(pcb->_report->contacts.capacity() + numContacts); // why capacity instead of size ?
              }
              FOREACH(itcontact, tmpReport.contacts) {
                pcb->_report->contacts.push_back(*itcontact);
              }
              // pcb->_report->contacts.swap(_report.contacts); // would be faster but seems just wrong...

              // Yes, that's superfluous in the link-link collision... but do we really want to duplicate all the code to optimize that ?
              if( (_options & OpenRAVE::CO_AllLinkCollisions) && std::find(pcb->_report->vLinkColliding.begin(), pcb->_report->vLinkColliding.end(), linkPair) != pcb->_report->vLinkColliding.end()) {
                pcb->_report->vLinkColliding.push_back(linkPair);
              }
              pcb->_bCollision = true;
              return !(_options & OpenRAVE::CO_AllGeometryContacts); // stop checking collision
          }

          pcb->_bCollision = true;
          return true; // since the report is NULL, there is no reason to continue
        }

        return false; // keep checking collision
    }

    static bool NarrowPhaseCheckDistance(fcl::CollisionObject *o1, fcl::CollisionObject *o2, void *data)
    {
        // TODO
        return false;
    }

    static LinkPair MakeLinkPair(LinkConstPtr plink1, LinkConstPtr plink2)
    {
        if( plink1.get() < plink2.get() ) {
            return make_pair(plink1, plink2);
        } else {
            return make_pair(plink1, plink2);
        }
    }

  LinkConstPtr GetCollisionLink(fcl::CollisionObject const &collObj)
    {
      FCLSpace::KinBodyInfo::LINK *link_raw = static_cast<FCLSpace::KinBodyInfo::LINK *>(collObj.getUserData());
      if( link_raw != NULL ) {
        return link_raw->GetLink();
      }
      RAVELOG_WARN("fcl collision object does not have a link attached");
      return LinkConstPtr();
    }



    int _options;
    boost::shared_ptr<FCLSpace> _fclspace;
    int _numMaxContacts;
    std::string _userdatakey;
};


#endif
