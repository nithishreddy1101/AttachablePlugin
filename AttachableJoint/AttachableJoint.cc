#include <vector>

#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include <gz/common/Profiler.hh>

#include <sdf/Element.hh>

#include "gz/sim/components/DetachableJoint.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/System.hh"
#include "gz/sim/components/Link.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/ParentEntity.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/Util.hh"

#include <string>
#include <iostream>

#include <gz/msgs/stringmsg.pb.h>
#include <gz/msgs/int32.pb.h>

#include "AttachableJoint.hh"
//#include "/home/david/Attacher/src/linking_try/include/AttachableJoint.hh"
#include <gz/sim/System.hh>

using namespace attachable_joint;

//using namespace gzition;
//using namespace gazebo;
//using namespace systems;

////////////////////////////////////////////////

/////////////////////////////////////////////////
void AttachableJoint::Configure(const gz::sim::Entity &_entity,
               const std::shared_ptr<const sdf::Element> &_sdf,
               gz::sim::EntityComponentManager &_ecm,
               gz::sim::EventManager &/*_eventMgr*/)
{
  ///Topics
  if (_sdf->HasElement("attachtopic")) 
  {
    this->attachtopic = _sdf->Get<std::string>("attachtopic");
  }
  else
  {
    this->attachtopic = "AttachableJoint";
    gzmsg << "'attachtopic' is 'AttachableJoint' by default.\n";
  }
  
  this->suppressChildWarning =
      _sdf->Get<bool>("suppress_child_warning", this->suppressChildWarning)
          .first;

  this->suppressParentWarning =
      _sdf->Get<bool>("suppress_parent_warning", this->suppressParentWarning)
          .first;
  this->validConfig = true;
}



////////////////////////////////////////////////// 
void AttachableJoint::PreUpdate(
  const gz::sim::UpdateInfo &/*_info*/,
  gz::sim::EntityComponentManager &_ecm)
{
  //gzmsg << "loop"<< std::endl;
  GZ_PROFILE("AttachableJoint::PreUpdate"); 
  gz::msgs::Int32 msg;

  if (this->not_initialized)
  {
    this->node.Subscribe(
        this->attachtopic, &AttachableJoint::OnAttachRequest, this);

    gzmsg << "AttachableJoint subscribing to messages on "
          << "[" << this->attachtopic << "]" << std::endl;

    this->not_initialized = false;

    ///////////////

    this->error_topic.reset();
    this->error_topic = this->node.Advertise<gz::msgs::Int32>("AttachableJoint/error");

    ///////////////

  }

  if (this->validConfig && this->attachRequested)
  {
    
    bool createNewAttachableJoint = true;  
      
    for(auto& item: this->attachableJointList)
    {
      if(item.second == this->attachableJointName)
      {
        createNewAttachableJoint = false;
        break;
      }
    }
    if (createNewAttachableJoint == true)
    {
      gz::sim::Entity pmodelEntity{gz::sim::kNullEntity};
        
      pmodelEntity = _ecm.EntityByComponents(gz::sim::components::Model(), gz::sim::components::Name(this->parentModelName));

      if (gz::sim::kNullEntity != pmodelEntity)
      {
        this->parentLinkEntity = _ecm.EntityByComponents(
            gz::sim::components::Link(), gz::sim::components::ParentEntity(pmodelEntity),
            gz::sim::components::Name(this->parentLinkName));
      
        if (gz::sim::kNullEntity != this->parentLinkEntity)
        {
          //Hacemos todo con el hijo
          gz::sim::Entity cmodelEntity{gz::sim::kNullEntity};
          
          cmodelEntity = _ecm.EntityByComponents(gz::sim::components::Model(), gz::sim::components::Name(this->childModelName));
          if (gz::sim::kNullEntity != cmodelEntity)
          {
            this->childLinkEntity = _ecm.EntityByComponents(
                gz::sim::components::Link(), gz::sim::components::ParentEntity(cmodelEntity),
                gz::sim::components::Name(this->childLinkName));
                
            if (gz::sim::kNullEntity != this->childLinkEntity)
            {
              // Attach the models
              // We do this by creating a detachable joint entity.
              this->attachableJointEntity = _ecm.CreateEntity();
              _ecm.CreateComponent(
                  this->attachableJointEntity,
                  gz::sim::components::DetachableJoint({this->parentLinkEntity,
                                              this->childLinkEntity, "fixed"})); 

              this->attachableJointList.push_back({this->attachableJointEntity, this->attachableJointName});
              this->initialized = true;
              this->attachRequested = false;
              msg.set_data(0);
            }
            else
            {
              this->attachRequested = false;
              gzwarn << "Child Link " << this->childLinkName
                      << " could not be found.\n";
              msg.set_data(1);
            }
          }
          else if (!this->suppressChildWarning)
          {
            this->attachRequested = false;
            gzwarn << "Child Model " << this->childModelName
                    << " could not be found.\n";
            msg.set_data(1);
          }
            //Hacemos todo con el hijo
        }
        else
        {
          this->attachRequested = false;
          gzwarn << "Parent Link " << this->parentLinkName
                  << " could not be found.\n";
          msg.set_data(1);
        }
      }
      else if (!this->suppressParentWarning)
      {
        gzwarn << "Parent Model " << this->parentModelName
                << " could not be found.\n"; //gzwarngzerr
        this->attachRequested = false;
        msg.set_data(1);
      }
    }
    else
    {
      this->attachRequested = false;
      msg.set_data(2);
    }
  }
  if (this->initialized)
  {
    if (this->detachRequested)
    {
      // Detach the models
      int i;
      msg.set_data(1);
      for(i=0;i<this->attachableJointList.size();i++)      
      {
        if(this->attachableJointList[i].second == this->attachableJointName)
        {
          msg.set_data(0);
          // gzdbg << "Removing entity: " << this->attachableJointList << std::endl;
          _ecm.RequestRemoveEntity(this->attachableJointList[i].first);

          this->attachableJointList.erase(this->attachableJointList.begin()+i);
          this->detachRequested = false;
          break;
        }
      }

    }
  }
  this->error_topic->Publish(msg);

}
 
//////////////////////////////////////////////////
void AttachableJoint::OnAttachRequest(const gz::msgs::StringMsg &msg)
{
  gzmsg << "El mensaje enviado es: " << msg.data() << std::endl;
  
  // [parentModel][ParentLink][ChildModel][ChildLink]
  //Now the Link must be nammed AttachableLink_Name or wont work

  std::string str = msg.data();//"[box1][box_body][box2][box_body]";
  std::string request;

  unsigned first = str.find('[');
  str = &str[first];
  unsigned last = str.find(']');
  this->parentModelName = str.substr(1,last-1);
  str = &str[last];

  first = str.find('[');
  str = &str[first];
  last = str.find(']');
  this->parentLinkName = str.substr(1,last-1);
  str = &str[last];

  first = str.find('[');
  str = &str[first];
  last = str.find(']');
  this->childModelName = str.substr(1,last-1);
  str = &str[last];

  first = str.find('[');
  str = &str[first];
  last = str.find(']');
  this->childLinkName = str.substr(1,last-1);
  str = &str[last];

  first = str.find('[');
  str = &str[first];
  last = str.find(']');
  request = str.substr(1,last-1);

  this->attachableJointName = this->parentModelName + "_" + this->parentLinkName + "_" + this->childModelName + "_" + this->childLinkName;
  
  if ("attach" == request)
  {
    this->attachRequested = true;

    gzmsg << "PM: " <<this->parentModelName <<" PL: "<< this->parentLinkName <<" CM: "<< this->childModelName <<" CL: "<< this->childLinkName 
           << std::endl << "\n\n\n";

  }
  else if ("detach" == request)
  {
    if (false == this->not_initialized)
    {
      this->detachRequested = true;

      gzmsg << "PM: " <<this->parentModelName <<" PL: "<< this->parentLinkName <<" CM: "<< this->childModelName <<" CL: "<< this->childLinkName 
            << std::endl << "\n\n\n";
    }
    else
    {
      gzmsg << "There is no AttachableJoint created yet";
    }
  }
  
  /*
  if ( (this->parentLinkName.find("AttachableLink") != -1) && (this->childLinkName.find("AttachableLink") != -1) ) {

    this->attachRequested = true;
    gzmsg << "PM: " <<this->parentModelName <<" PL: "<< this->parentLinkName <<" CM: "<< this->childModelName <<" CL: "<< this->childLinkName 
           << std::endl;
  } 
  else {
      gzerr << "parent link or child link are not AttachableLinks"<< std::endl;
  }
  */

}


GZ_ADD_PLUGIN(attachable_joint::AttachableJoint,
                    gz::sim::System,
                    attachable_joint::AttachableJoint::ISystemConfigure,
                    attachable_joint::AttachableJoint::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(AttachableJoint,"attachable_joint::AtachableJoint")
