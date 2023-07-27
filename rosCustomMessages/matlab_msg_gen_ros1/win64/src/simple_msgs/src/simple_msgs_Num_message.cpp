// Copyright 2019-2021 The MathWorks, Inc.
// Common copy functions for simple_msgs/Num
#include "boost/date_time.hpp"
#include "boost/shared_array.hpp"
#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable : 4244)
#pragma warning(disable : 4265)
#pragma warning(disable : 4458)
#pragma warning(disable : 4100)
#pragma warning(disable : 4127)
#pragma warning(disable : 4267)
#pragma warning(disable : 4068)
#pragma warning(disable : 4245)
#else
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wunused-local-typedefs"
#pragma GCC diagnostic ignored "-Wredundant-decls"
#pragma GCC diagnostic ignored "-Wnon-virtual-dtor"
#pragma GCC diagnostic ignored "-Wdelete-non-virtual-dtor"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wunused-variable"
#pragma GCC diagnostic ignored "-Wshadow"
#endif //_MSC_VER
#include "ros/ros.h"
#include "simple_msgs/Num.h"
#include "visibility_control.h"
#include "MATLABROSMsgInterface.hpp"
#include "ROSPubSubTemplates.hpp"
class SIMPLE_MSGS_EXPORT simple_msgs_msg_Num_common : public MATLABROSMsgInterface<simple_msgs::Num> {
  public:
    virtual ~simple_msgs_msg_Num_common(){}
    virtual void copy_from_struct(simple_msgs::Num* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const simple_msgs::Num* msg, MultiLibLoader loader, size_t size = 1);
};
  void simple_msgs_msg_Num_common::copy_from_struct(simple_msgs::Num* msg, const matlab::data::Struct& arr,
               MultiLibLoader loader) {
    try {
        //num
        const matlab::data::TypedArray<int64_t> num_arr = arr["Num_"];
        msg->num = num_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Num_' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Num_' is wrong type; expected a int64.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T simple_msgs_msg_Num_common::get_arr(MDFactory_T& factory, const simple_msgs::Num* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","Num_"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("simple_msgs/Num");
    // num
    auto currentElement_num = (msg + ctr)->num;
    outArray[ctr]["Num_"] = factory.createScalar(currentElement_num);
    }
    return std::move(outArray);
  } 
class SIMPLE_MSGS_EXPORT simple_msgs_Num_message : public ROSMsgElementInterfaceFactory {
  public:
    virtual ~simple_msgs_Num_message(){}
    virtual std::shared_ptr<MATLABPublisherInterface> generatePublisherInterface(ElementType type);
    virtual std::shared_ptr<MATLABSubscriberInterface> generateSubscriberInterface(ElementType type);
    virtual std::shared_ptr<MATLABRosbagWriterInterface> generateRosbagWriterInterface(ElementType type);
};  
  std::shared_ptr<MATLABPublisherInterface> 
          simple_msgs_Num_message::generatePublisherInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSPublisherImpl<simple_msgs::Num,simple_msgs_msg_Num_common>>();
  }
  std::shared_ptr<MATLABSubscriberInterface> 
         simple_msgs_Num_message::generateSubscriberInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSSubscriberImpl<simple_msgs::Num,simple_msgs::Num::ConstPtr,simple_msgs_msg_Num_common>>();
  }
#include "ROSbagTemplates.hpp" 
  std::shared_ptr<MATLABRosbagWriterInterface>
         simple_msgs_Num_message::generateRosbagWriterInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSBagWriterImpl<simple_msgs::Num,simple_msgs_msg_Num_common>>();
  }
#include "register_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(simple_msgs_msg_Num_common, MATLABROSMsgInterface<simple_msgs::Num>)
CLASS_LOADER_REGISTER_CLASS(simple_msgs_Num_message, ROSMsgElementInterfaceFactory)
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER
//gen-1