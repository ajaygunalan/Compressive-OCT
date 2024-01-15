// Copyright 2019-2021 The MathWorks, Inc.
// Common copy functions for simple_msgs/AddTwoIntsRequest
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
#include "simple_msgs/AddTwoInts.h"
#include "visibility_control.h"
#include "ROSPubSubTemplates.hpp"
#include "ROSServiceTemplates.hpp"
class SIMPLE_MSGS_EXPORT simple_msgs_msg_AddTwoIntsRequest_common : public MATLABROSMsgInterface<simple_msgs::AddTwoInts::Request> {
  public:
    virtual ~simple_msgs_msg_AddTwoIntsRequest_common(){}
    virtual void copy_from_struct(simple_msgs::AddTwoInts::Request* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const simple_msgs::AddTwoInts::Request* msg, MultiLibLoader loader, size_t size = 1);
};
  void simple_msgs_msg_AddTwoIntsRequest_common::copy_from_struct(simple_msgs::AddTwoInts::Request* msg, const matlab::data::Struct& arr,
               MultiLibLoader loader) {
    try {
        //a
        const matlab::data::TypedArray<int64_t> a_arr = arr["A"];
        msg->a = a_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'A' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'A' is wrong type; expected a int64.");
    }
    try {
        //b
        const matlab::data::TypedArray<int64_t> b_arr = arr["B"];
        msg->b = b_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'B' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'B' is wrong type; expected a int64.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T simple_msgs_msg_AddTwoIntsRequest_common::get_arr(MDFactory_T& factory, const simple_msgs::AddTwoInts::Request* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","A","B"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("simple_msgs/AddTwoIntsRequest");
    // a
    auto currentElement_a = (msg + ctr)->a;
    outArray[ctr]["A"] = factory.createScalar(currentElement_a);
    // b
    auto currentElement_b = (msg + ctr)->b;
    outArray[ctr]["B"] = factory.createScalar(currentElement_b);
    }
    return std::move(outArray);
  }
class SIMPLE_MSGS_EXPORT simple_msgs_msg_AddTwoIntsResponse_common : public MATLABROSMsgInterface<simple_msgs::AddTwoInts::Response> {
  public:
    virtual ~simple_msgs_msg_AddTwoIntsResponse_common(){}
    virtual void copy_from_struct(simple_msgs::AddTwoInts::Response* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const simple_msgs::AddTwoInts::Response* msg, MultiLibLoader loader, size_t size = 1);
};
  void simple_msgs_msg_AddTwoIntsResponse_common::copy_from_struct(simple_msgs::AddTwoInts::Response* msg, const matlab::data::Struct& arr,
               MultiLibLoader loader) {
    try {
        //sum
        const matlab::data::TypedArray<int64_t> sum_arr = arr["Sum"];
        msg->sum = sum_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Sum' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Sum' is wrong type; expected a int64.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T simple_msgs_msg_AddTwoIntsResponse_common::get_arr(MDFactory_T& factory, const simple_msgs::AddTwoInts::Response* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","Sum"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("simple_msgs/AddTwoIntsResponse");
    // sum
    auto currentElement_sum = (msg + ctr)->sum;
    outArray[ctr]["Sum"] = factory.createScalar(currentElement_sum);
    }
    return std::move(outArray);
  } 
class SIMPLE_MSGS_EXPORT simple_msgs_AddTwoInts_service : public ROSMsgElementInterfaceFactory {
  public:
    virtual ~simple_msgs_AddTwoInts_service(){}
    virtual std::shared_ptr<MATLABPublisherInterface> generatePublisherInterface(ElementType type);
    virtual std::shared_ptr<MATLABSubscriberInterface> generateSubscriberInterface(ElementType type);
    virtual std::shared_ptr<MATLABRosbagWriterInterface> generateRosbagWriterInterface(ElementType type);
    virtual std::shared_ptr<MATLABSvcServerInterface> generateSvcServerInterface();
    virtual std::shared_ptr<MATLABSvcClientInterface> generateSvcClientInterface();
};  
  std::shared_ptr<MATLABPublisherInterface> 
          simple_msgs_AddTwoInts_service::generatePublisherInterface(ElementType type){
    std::shared_ptr<MATLABPublisherInterface> ptr;
    if(type == eRequest){
        ptr = std::make_shared<ROSPublisherImpl<simple_msgs::AddTwoInts::Request,simple_msgs_msg_AddTwoIntsRequest_common>>();
    }else if(type == eResponse){
        ptr = std::make_shared<ROSPublisherImpl<simple_msgs::AddTwoInts::Response,simple_msgs_msg_AddTwoIntsResponse_common>>();
    }else{
        throw std::invalid_argument("Wrong input, Expected 'Request' or 'Response'");
    }
    return ptr;
  }
  std::shared_ptr<MATLABSubscriberInterface> 
          simple_msgs_AddTwoInts_service::generateSubscriberInterface(ElementType type){
    std::shared_ptr<MATLABSubscriberInterface> ptr;
    if(type == eRequest){
        ptr = std::make_shared<ROSSubscriberImpl<simple_msgs::AddTwoInts::Request,simple_msgs::AddTwoInts::Request::ConstPtr,simple_msgs_msg_AddTwoIntsRequest_common>>();
    }else if(type == eResponse){
        ptr = std::make_shared<ROSSubscriberImpl<simple_msgs::AddTwoInts::Response,simple_msgs::AddTwoInts::Response::ConstPtr,simple_msgs_msg_AddTwoIntsResponse_common>>();
    }else{
        throw std::invalid_argument("Wrong input, Expected 'Request' or 'Response'");
    }
    return ptr;
  }
  std::shared_ptr<MATLABSvcServerInterface> 
          simple_msgs_AddTwoInts_service::generateSvcServerInterface(){
    return std::make_shared<ROSSvcServerImpl<simple_msgs::AddTwoInts::Request,simple_msgs::AddTwoInts::Response,simple_msgs_msg_AddTwoIntsRequest_common,simple_msgs_msg_AddTwoIntsResponse_common>>();
  }
  std::shared_ptr<MATLABSvcClientInterface> 
          simple_msgs_AddTwoInts_service::generateSvcClientInterface(){
    return std::make_shared<ROSSvcClientImpl<simple_msgs::AddTwoInts,simple_msgs::AddTwoInts::Request,simple_msgs::AddTwoInts::Response,simple_msgs_msg_AddTwoIntsRequest_common,simple_msgs_msg_AddTwoIntsResponse_common>>();
  }
#include "ROSbagTemplates.hpp" 
  std::shared_ptr<MATLABRosbagWriterInterface> 
          simple_msgs_AddTwoInts_service::generateRosbagWriterInterface(ElementType type){
    std::shared_ptr<MATLABRosbagWriterInterface> ptr;
    if(type == eRequest){
        ptr = std::make_shared<ROSBagWriterImpl<simple_msgs::AddTwoIntsRequest,simple_msgs_msg_AddTwoIntsRequest_common>>();
    }else if(type == eResponse){
        ptr = std::make_shared<ROSBagWriterImpl<simple_msgs::AddTwoIntsResponse,simple_msgs_msg_AddTwoIntsResponse_common>>();
    }else{
        throw std::invalid_argument("Wrong input, Expected 'Request' or 'Response'");
    }
    return ptr;
  }
#include "register_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(simple_msgs_msg_AddTwoIntsRequest_common, MATLABROSMsgInterface<simple_msgs::AddTwoIntsRequest>)
CLASS_LOADER_REGISTER_CLASS(simple_msgs_msg_AddTwoIntsResponse_common, MATLABROSMsgInterface<simple_msgs::AddTwoIntsResponse>)
CLASS_LOADER_REGISTER_CLASS(simple_msgs_AddTwoInts_service, ROSMsgElementInterfaceFactory)
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER
//gen-1
