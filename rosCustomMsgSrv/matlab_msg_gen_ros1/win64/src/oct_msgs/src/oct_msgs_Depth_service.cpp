// Copyright 2019-2021 The MathWorks, Inc.
// Common copy functions for oct_msgs/DepthRequest
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
#include "oct_msgs/Depth.h"
#include "visibility_control.h"
#include "ROSPubSubTemplates.hpp"
#include "ROSServiceTemplates.hpp"
class OCT_MSGS_EXPORT oct_msgs_msg_DepthRequest_common : public MATLABROSMsgInterface<oct_msgs::Depth::Request> {
  public:
    virtual ~oct_msgs_msg_DepthRequest_common(){}
    virtual void copy_from_struct(oct_msgs::Depth::Request* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const oct_msgs::Depth::Request* msg, MultiLibLoader loader, size_t size = 1);
};
  void oct_msgs_msg_DepthRequest_common::copy_from_struct(oct_msgs::Depth::Request* msg, const matlab::data::Struct& arr,
               MultiLibLoader loader) {
  }
  //----------------------------------------------------------------------------
  MDArray_T oct_msgs_msg_DepthRequest_common::get_arr(MDFactory_T& factory, const oct_msgs::Depth::Request* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("oct_msgs/DepthRequest");
    }
    return std::move(outArray);
  }
class OCT_MSGS_EXPORT oct_msgs_msg_DepthResponse_common : public MATLABROSMsgInterface<oct_msgs::Depth::Response> {
  public:
    virtual ~oct_msgs_msg_DepthResponse_common(){}
    virtual void copy_from_struct(oct_msgs::Depth::Response* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const oct_msgs::Depth::Response* msg, MultiLibLoader loader, size_t size = 1);
};
  void oct_msgs_msg_DepthResponse_common::copy_from_struct(oct_msgs::Depth::Response* msg, const matlab::data::Struct& arr,
               MultiLibLoader loader) {
    try {
        //depth
        const matlab::data::StructArray depth_arr = arr["Depth"];
        auto msgClassPtr_depth = getCommonObject<std_msgs::Float64>("std_msgs_msg_Float64_common",loader);
        msgClassPtr_depth->copy_from_struct(&msg->depth,depth_arr[0],loader);
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Depth' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Depth' is wrong type; expected a struct.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T oct_msgs_msg_DepthResponse_common::get_arr(MDFactory_T& factory, const oct_msgs::Depth::Response* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","Depth"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("oct_msgs/DepthResponse");
    // depth
    auto currentElement_depth = (msg + ctr)->depth;
    auto msgClassPtr_depth = getCommonObject<std_msgs::Float64>("std_msgs_msg_Float64_common",loader);
    outArray[ctr]["Depth"] = msgClassPtr_depth->get_arr(factory, &currentElement_depth, loader);
    }
    return std::move(outArray);
  } 
class OCT_MSGS_EXPORT oct_msgs_Depth_service : public ROSMsgElementInterfaceFactory {
  public:
    virtual ~oct_msgs_Depth_service(){}
    virtual std::shared_ptr<MATLABPublisherInterface> generatePublisherInterface(ElementType type);
    virtual std::shared_ptr<MATLABSubscriberInterface> generateSubscriberInterface(ElementType type);
    virtual std::shared_ptr<MATLABRosbagWriterInterface> generateRosbagWriterInterface(ElementType type);
    virtual std::shared_ptr<MATLABSvcServerInterface> generateSvcServerInterface();
    virtual std::shared_ptr<MATLABSvcClientInterface> generateSvcClientInterface();
};  
  std::shared_ptr<MATLABPublisherInterface> 
          oct_msgs_Depth_service::generatePublisherInterface(ElementType type){
    std::shared_ptr<MATLABPublisherInterface> ptr;
    if(type == eRequest){
        ptr = std::make_shared<ROSPublisherImpl<oct_msgs::Depth::Request,oct_msgs_msg_DepthRequest_common>>();
    }else if(type == eResponse){
        ptr = std::make_shared<ROSPublisherImpl<oct_msgs::Depth::Response,oct_msgs_msg_DepthResponse_common>>();
    }else{
        throw std::invalid_argument("Wrong input, Expected 'Request' or 'Response'");
    }
    return ptr;
  }
  std::shared_ptr<MATLABSubscriberInterface> 
          oct_msgs_Depth_service::generateSubscriberInterface(ElementType type){
    std::shared_ptr<MATLABSubscriberInterface> ptr;
    if(type == eRequest){
        ptr = std::make_shared<ROSSubscriberImpl<oct_msgs::Depth::Request,oct_msgs::Depth::Request::ConstPtr,oct_msgs_msg_DepthRequest_common>>();
    }else if(type == eResponse){
        ptr = std::make_shared<ROSSubscriberImpl<oct_msgs::Depth::Response,oct_msgs::Depth::Response::ConstPtr,oct_msgs_msg_DepthResponse_common>>();
    }else{
        throw std::invalid_argument("Wrong input, Expected 'Request' or 'Response'");
    }
    return ptr;
  }
  std::shared_ptr<MATLABSvcServerInterface> 
          oct_msgs_Depth_service::generateSvcServerInterface(){
    return std::make_shared<ROSSvcServerImpl<oct_msgs::Depth::Request,oct_msgs::Depth::Response,oct_msgs_msg_DepthRequest_common,oct_msgs_msg_DepthResponse_common>>();
  }
  std::shared_ptr<MATLABSvcClientInterface> 
          oct_msgs_Depth_service::generateSvcClientInterface(){
    return std::make_shared<ROSSvcClientImpl<oct_msgs::Depth,oct_msgs::Depth::Request,oct_msgs::Depth::Response,oct_msgs_msg_DepthRequest_common,oct_msgs_msg_DepthResponse_common>>();
  }
#include "ROSbagTemplates.hpp" 
  std::shared_ptr<MATLABRosbagWriterInterface> 
          oct_msgs_Depth_service::generateRosbagWriterInterface(ElementType type){
    std::shared_ptr<MATLABRosbagWriterInterface> ptr;
    if(type == eRequest){
        ptr = std::make_shared<ROSBagWriterImpl<oct_msgs::DepthRequest,oct_msgs_msg_DepthRequest_common>>();
    }else if(type == eResponse){
        ptr = std::make_shared<ROSBagWriterImpl<oct_msgs::DepthResponse,oct_msgs_msg_DepthResponse_common>>();
    }else{
        throw std::invalid_argument("Wrong input, Expected 'Request' or 'Response'");
    }
    return ptr;
  }
#include "register_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(oct_msgs_msg_DepthRequest_common, MATLABROSMsgInterface<oct_msgs::DepthRequest>)
CLASS_LOADER_REGISTER_CLASS(oct_msgs_msg_DepthResponse_common, MATLABROSMsgInterface<oct_msgs::DepthResponse>)
CLASS_LOADER_REGISTER_CLASS(oct_msgs_Depth_service, ROSMsgElementInterfaceFactory)
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER
//gen-1
