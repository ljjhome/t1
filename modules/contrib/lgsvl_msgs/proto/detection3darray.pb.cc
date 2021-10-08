// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: modules/contrib/lgsvl_msgs/proto/detection3darray.proto

#include "modules/contrib/lgsvl_msgs/proto/detection3darray.pb.h"

#include <algorithm>

#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/extension_set.h>
#include <google/protobuf/wire_format_lite.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/reflection_ops.h>
#include <google/protobuf/wire_format.h>
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>

PROTOBUF_PRAGMA_INIT_SEG
namespace apollo {
namespace contrib {
namespace lgsvl_msgs {
constexpr Detection3DArray::Detection3DArray(
  ::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized)
  : detections_()
  , header_(nullptr){}
struct Detection3DArrayDefaultTypeInternal {
  constexpr Detection3DArrayDefaultTypeInternal()
    : _instance(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized{}) {}
  ~Detection3DArrayDefaultTypeInternal() {}
  union {
    Detection3DArray _instance;
  };
};
PROTOBUF_ATTRIBUTE_NO_DESTROY PROTOBUF_CONSTINIT Detection3DArrayDefaultTypeInternal _Detection3DArray_default_instance_;
}  // namespace lgsvl_msgs
}  // namespace contrib
}  // namespace apollo
static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_modules_2fcontrib_2flgsvl_5fmsgs_2fproto_2fdetection3darray_2eproto[1];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_modules_2fcontrib_2flgsvl_5fmsgs_2fproto_2fdetection3darray_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_modules_2fcontrib_2flgsvl_5fmsgs_2fproto_2fdetection3darray_2eproto = nullptr;

const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_modules_2fcontrib_2flgsvl_5fmsgs_2fproto_2fdetection3darray_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  ~0u,  // no _has_bits_
  PROTOBUF_FIELD_OFFSET(::apollo::contrib::lgsvl_msgs::Detection3DArray, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  ~0u,  // no _inlined_string_donated_
  PROTOBUF_FIELD_OFFSET(::apollo::contrib::lgsvl_msgs::Detection3DArray, header_),
  PROTOBUF_FIELD_OFFSET(::apollo::contrib::lgsvl_msgs::Detection3DArray, detections_),
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, -1, -1, sizeof(::apollo::contrib::lgsvl_msgs::Detection3DArray)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::apollo::contrib::lgsvl_msgs::_Detection3DArray_default_instance_),
};

const char descriptor_table_protodef_modules_2fcontrib_2flgsvl_5fmsgs_2fproto_2fdetection3darray_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n7modules/contrib/lgsvl_msgs/proto/detec"
  "tion3darray.proto\022\031apollo.contrib.lgsvl_"
  "msgs\032!modules/common/proto/header.proto\032"
  "2modules/contrib/lgsvl_msgs/proto/detect"
  "ion3d.proto\"u\n\020Detection3DArray\022%\n\006heade"
  "r\030\001 \001(\0132\025.apollo.common.Header\022:\n\ndetect"
  "ions\030\002 \003(\0132&.apollo.contrib.lgsvl_msgs.D"
  "etection3Db\006proto3"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_modules_2fcontrib_2flgsvl_5fmsgs_2fproto_2fdetection3darray_2eproto_deps[2] = {
  &::descriptor_table_modules_2fcommon_2fproto_2fheader_2eproto,
  &::descriptor_table_modules_2fcontrib_2flgsvl_5fmsgs_2fproto_2fdetection3d_2eproto,
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_modules_2fcontrib_2flgsvl_5fmsgs_2fproto_2fdetection3darray_2eproto_once;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_modules_2fcontrib_2flgsvl_5fmsgs_2fproto_2fdetection3darray_2eproto = {
  false, false, 298, descriptor_table_protodef_modules_2fcontrib_2flgsvl_5fmsgs_2fproto_2fdetection3darray_2eproto, "modules/contrib/lgsvl_msgs/proto/detection3darray.proto", 
  &descriptor_table_modules_2fcontrib_2flgsvl_5fmsgs_2fproto_2fdetection3darray_2eproto_once, descriptor_table_modules_2fcontrib_2flgsvl_5fmsgs_2fproto_2fdetection3darray_2eproto_deps, 2, 1,
  schemas, file_default_instances, TableStruct_modules_2fcontrib_2flgsvl_5fmsgs_2fproto_2fdetection3darray_2eproto::offsets,
  file_level_metadata_modules_2fcontrib_2flgsvl_5fmsgs_2fproto_2fdetection3darray_2eproto, file_level_enum_descriptors_modules_2fcontrib_2flgsvl_5fmsgs_2fproto_2fdetection3darray_2eproto, file_level_service_descriptors_modules_2fcontrib_2flgsvl_5fmsgs_2fproto_2fdetection3darray_2eproto,
};
PROTOBUF_ATTRIBUTE_WEAK const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable* descriptor_table_modules_2fcontrib_2flgsvl_5fmsgs_2fproto_2fdetection3darray_2eproto_getter() {
  return &descriptor_table_modules_2fcontrib_2flgsvl_5fmsgs_2fproto_2fdetection3darray_2eproto;
}

// Force running AddDescriptors() at dynamic initialization time.
PROTOBUF_ATTRIBUTE_INIT_PRIORITY static ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptorsRunner dynamic_init_dummy_modules_2fcontrib_2flgsvl_5fmsgs_2fproto_2fdetection3darray_2eproto(&descriptor_table_modules_2fcontrib_2flgsvl_5fmsgs_2fproto_2fdetection3darray_2eproto);
namespace apollo {
namespace contrib {
namespace lgsvl_msgs {

// ===================================================================

class Detection3DArray::_Internal {
 public:
  static const ::apollo::common::Header& header(const Detection3DArray* msg);
};

const ::apollo::common::Header&
Detection3DArray::_Internal::header(const Detection3DArray* msg) {
  return *msg->header_;
}
void Detection3DArray::clear_header() {
  if (GetArenaForAllocation() == nullptr && header_ != nullptr) {
    delete header_;
  }
  header_ = nullptr;
}
void Detection3DArray::clear_detections() {
  detections_.Clear();
}
Detection3DArray::Detection3DArray(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                         bool is_message_owned)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena, is_message_owned),
  detections_(arena) {
  SharedCtor();
  if (!is_message_owned) {
    RegisterArenaDtor(arena);
  }
  // @@protoc_insertion_point(arena_constructor:apollo.contrib.lgsvl_msgs.Detection3DArray)
}
Detection3DArray::Detection3DArray(const Detection3DArray& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      detections_(from.detections_) {
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  if (from._internal_has_header()) {
    header_ = new ::apollo::common::Header(*from.header_);
  } else {
    header_ = nullptr;
  }
  // @@protoc_insertion_point(copy_constructor:apollo.contrib.lgsvl_msgs.Detection3DArray)
}

void Detection3DArray::SharedCtor() {
header_ = nullptr;
}

Detection3DArray::~Detection3DArray() {
  // @@protoc_insertion_point(destructor:apollo.contrib.lgsvl_msgs.Detection3DArray)
  if (GetArenaForAllocation() != nullptr) return;
  SharedDtor();
  _internal_metadata_.Delete<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

inline void Detection3DArray::SharedDtor() {
  GOOGLE_DCHECK(GetArenaForAllocation() == nullptr);
  if (this != internal_default_instance()) delete header_;
}

void Detection3DArray::ArenaDtor(void* object) {
  Detection3DArray* _this = reinterpret_cast< Detection3DArray* >(object);
  (void)_this;
}
void Detection3DArray::RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena*) {
}
void Detection3DArray::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}

void Detection3DArray::Clear() {
// @@protoc_insertion_point(message_clear_start:apollo.contrib.lgsvl_msgs.Detection3DArray)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  detections_.Clear();
  if (GetArenaForAllocation() == nullptr && header_ != nullptr) {
    delete header_;
  }
  header_ = nullptr;
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* Detection3DArray::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    switch (tag >> 3) {
      // .apollo.common.Header header = 1;
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 10)) {
          ptr = ctx->ParseMessage(_internal_mutable_header(), ptr);
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // repeated .apollo.contrib.lgsvl_msgs.Detection3D detections = 2;
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 18)) {
          ptr -= 1;
          do {
            ptr += 1;
            ptr = ctx->ParseMessage(_internal_add_detections(), ptr);
            CHK_(ptr);
            if (!ctx->DataAvailable(ptr)) break;
          } while (::PROTOBUF_NAMESPACE_ID::internal::ExpectTag<18>(ptr));
        } else
          goto handle_unusual;
        continue;
      default:
        goto handle_unusual;
    }  // switch
  handle_unusual:
    if ((tag == 0) || ((tag & 7) == 4)) {
      CHK_(ptr);
      ctx->SetLastTag(tag);
      goto message_done;
    }
    ptr = UnknownFieldParse(
        tag,
        _internal_metadata_.mutable_unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(),
        ptr, ctx);
    CHK_(ptr != nullptr);
  }  // while
message_done:
  return ptr;
failure:
  ptr = nullptr;
  goto message_done;
#undef CHK_
}

::PROTOBUF_NAMESPACE_ID::uint8* Detection3DArray::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:apollo.contrib.lgsvl_msgs.Detection3DArray)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // .apollo.common.Header header = 1;
  if (this->_internal_has_header()) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(
        1, _Internal::header(this), target, stream);
  }

  // repeated .apollo.contrib.lgsvl_msgs.Detection3D detections = 2;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->_internal_detections_size()); i < n; i++) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(2, this->_internal_detections(i), target, stream);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:apollo.contrib.lgsvl_msgs.Detection3DArray)
  return target;
}

size_t Detection3DArray::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:apollo.contrib.lgsvl_msgs.Detection3DArray)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // repeated .apollo.contrib.lgsvl_msgs.Detection3D detections = 2;
  total_size += 1UL * this->_internal_detections_size();
  for (const auto& msg : this->detections_) {
    total_size +=
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(msg);
  }

  // .apollo.common.Header header = 1;
  if (this->_internal_has_header()) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
        *header_);
  }

  return MaybeComputeUnknownFieldsSize(total_size, &_cached_size_);
}

const ::PROTOBUF_NAMESPACE_ID::Message::ClassData Detection3DArray::_class_data_ = {
    ::PROTOBUF_NAMESPACE_ID::Message::CopyWithSizeCheck,
    Detection3DArray::MergeImpl
};
const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*Detection3DArray::GetClassData() const { return &_class_data_; }

void Detection3DArray::MergeImpl(::PROTOBUF_NAMESPACE_ID::Message* to,
                      const ::PROTOBUF_NAMESPACE_ID::Message& from) {
  static_cast<Detection3DArray *>(to)->MergeFrom(
      static_cast<const Detection3DArray &>(from));
}


void Detection3DArray::MergeFrom(const Detection3DArray& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:apollo.contrib.lgsvl_msgs.Detection3DArray)
  GOOGLE_DCHECK_NE(&from, this);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  detections_.MergeFrom(from.detections_);
  if (from._internal_has_header()) {
    _internal_mutable_header()->::apollo::common::Header::MergeFrom(from._internal_header());
  }
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
}

void Detection3DArray::CopyFrom(const Detection3DArray& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:apollo.contrib.lgsvl_msgs.Detection3DArray)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool Detection3DArray::IsInitialized() const {
  return true;
}

void Detection3DArray::InternalSwap(Detection3DArray* other) {
  using std::swap;
  _internal_metadata_.InternalSwap(&other->_internal_metadata_);
  detections_.InternalSwap(&other->detections_);
  swap(header_, other->header_);
}

::PROTOBUF_NAMESPACE_ID::Metadata Detection3DArray::GetMetadata() const {
  return ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(
      &descriptor_table_modules_2fcontrib_2flgsvl_5fmsgs_2fproto_2fdetection3darray_2eproto_getter, &descriptor_table_modules_2fcontrib_2flgsvl_5fmsgs_2fproto_2fdetection3darray_2eproto_once,
      file_level_metadata_modules_2fcontrib_2flgsvl_5fmsgs_2fproto_2fdetection3darray_2eproto[0]);
}

// @@protoc_insertion_point(namespace_scope)
}  // namespace lgsvl_msgs
}  // namespace contrib
}  // namespace apollo
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::apollo::contrib::lgsvl_msgs::Detection3DArray* Arena::CreateMaybeMessage< ::apollo::contrib::lgsvl_msgs::Detection3DArray >(Arena* arena) {
  return Arena::CreateMessageInternal< ::apollo::contrib::lgsvl_msgs::Detection3DArray >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
