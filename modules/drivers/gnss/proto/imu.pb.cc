// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: modules/drivers/gnss/proto/imu.proto

#include "modules/drivers/gnss/proto/imu.pb.h"

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
namespace drivers {
namespace gnss {
constexpr Imu::Imu(
  ::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized)
  : header_(nullptr)
  , linear_acceleration_(nullptr)
  , angular_velocity_(nullptr)
  , measurement_time_(0)
  , measurement_span_(0){}
struct ImuDefaultTypeInternal {
  constexpr ImuDefaultTypeInternal()
    : _instance(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized{}) {}
  ~ImuDefaultTypeInternal() {}
  union {
    Imu _instance;
  };
};
PROTOBUF_ATTRIBUTE_NO_DESTROY PROTOBUF_CONSTINIT ImuDefaultTypeInternal _Imu_default_instance_;
}  // namespace gnss
}  // namespace drivers
}  // namespace apollo
static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_modules_2fdrivers_2fgnss_2fproto_2fimu_2eproto[1];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_modules_2fdrivers_2fgnss_2fproto_2fimu_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_modules_2fdrivers_2fgnss_2fproto_2fimu_2eproto = nullptr;

const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_modules_2fdrivers_2fgnss_2fproto_2fimu_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  PROTOBUF_FIELD_OFFSET(::apollo::drivers::gnss::Imu, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::apollo::drivers::gnss::Imu, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  ~0u,  // no _inlined_string_donated_
  PROTOBUF_FIELD_OFFSET(::apollo::drivers::gnss::Imu, header_),
  PROTOBUF_FIELD_OFFSET(::apollo::drivers::gnss::Imu, measurement_time_),
  PROTOBUF_FIELD_OFFSET(::apollo::drivers::gnss::Imu, measurement_span_),
  PROTOBUF_FIELD_OFFSET(::apollo::drivers::gnss::Imu, linear_acceleration_),
  PROTOBUF_FIELD_OFFSET(::apollo::drivers::gnss::Imu, angular_velocity_),
  0,
  3,
  4,
  1,
  2,
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 11, -1, sizeof(::apollo::drivers::gnss::Imu)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::apollo::drivers::gnss::_Imu_default_instance_),
};

const char descriptor_table_protodef_modules_2fdrivers_2fgnss_2fproto_2fimu_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n$modules/drivers/gnss/proto/imu.proto\022\023"
  "apollo.drivers.gnss\032!modules/common/prot"
  "o/header.proto\032#modules/common/proto/geo"
  "metry.proto\"\312\001\n\003Imu\022%\n\006header\030\001 \001(\0132\025.ap"
  "ollo.common.Header\022\030\n\020measurement_time\030\002"
  " \001(\001\022\033\n\020measurement_span\030\003 \001(\002:\0010\0223\n\023lin"
  "ear_acceleration\030\004 \001(\0132\026.apollo.common.P"
  "oint3D\0220\n\020angular_velocity\030\005 \001(\0132\026.apoll"
  "o.common.Point3D"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_modules_2fdrivers_2fgnss_2fproto_2fimu_2eproto_deps[2] = {
  &::descriptor_table_modules_2fcommon_2fproto_2fgeometry_2eproto,
  &::descriptor_table_modules_2fcommon_2fproto_2fheader_2eproto,
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_modules_2fdrivers_2fgnss_2fproto_2fimu_2eproto_once;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_modules_2fdrivers_2fgnss_2fproto_2fimu_2eproto = {
  false, false, 336, descriptor_table_protodef_modules_2fdrivers_2fgnss_2fproto_2fimu_2eproto, "modules/drivers/gnss/proto/imu.proto", 
  &descriptor_table_modules_2fdrivers_2fgnss_2fproto_2fimu_2eproto_once, descriptor_table_modules_2fdrivers_2fgnss_2fproto_2fimu_2eproto_deps, 2, 1,
  schemas, file_default_instances, TableStruct_modules_2fdrivers_2fgnss_2fproto_2fimu_2eproto::offsets,
  file_level_metadata_modules_2fdrivers_2fgnss_2fproto_2fimu_2eproto, file_level_enum_descriptors_modules_2fdrivers_2fgnss_2fproto_2fimu_2eproto, file_level_service_descriptors_modules_2fdrivers_2fgnss_2fproto_2fimu_2eproto,
};
PROTOBUF_ATTRIBUTE_WEAK const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable* descriptor_table_modules_2fdrivers_2fgnss_2fproto_2fimu_2eproto_getter() {
  return &descriptor_table_modules_2fdrivers_2fgnss_2fproto_2fimu_2eproto;
}

// Force running AddDescriptors() at dynamic initialization time.
PROTOBUF_ATTRIBUTE_INIT_PRIORITY static ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptorsRunner dynamic_init_dummy_modules_2fdrivers_2fgnss_2fproto_2fimu_2eproto(&descriptor_table_modules_2fdrivers_2fgnss_2fproto_2fimu_2eproto);
namespace apollo {
namespace drivers {
namespace gnss {

// ===================================================================

class Imu::_Internal {
 public:
  using HasBits = decltype(std::declval<Imu>()._has_bits_);
  static const ::apollo::common::Header& header(const Imu* msg);
  static void set_has_header(HasBits* has_bits) {
    (*has_bits)[0] |= 1u;
  }
  static void set_has_measurement_time(HasBits* has_bits) {
    (*has_bits)[0] |= 8u;
  }
  static void set_has_measurement_span(HasBits* has_bits) {
    (*has_bits)[0] |= 16u;
  }
  static const ::apollo::common::Point3D& linear_acceleration(const Imu* msg);
  static void set_has_linear_acceleration(HasBits* has_bits) {
    (*has_bits)[0] |= 2u;
  }
  static const ::apollo::common::Point3D& angular_velocity(const Imu* msg);
  static void set_has_angular_velocity(HasBits* has_bits) {
    (*has_bits)[0] |= 4u;
  }
};

const ::apollo::common::Header&
Imu::_Internal::header(const Imu* msg) {
  return *msg->header_;
}
const ::apollo::common::Point3D&
Imu::_Internal::linear_acceleration(const Imu* msg) {
  return *msg->linear_acceleration_;
}
const ::apollo::common::Point3D&
Imu::_Internal::angular_velocity(const Imu* msg) {
  return *msg->angular_velocity_;
}
void Imu::clear_header() {
  if (header_ != nullptr) header_->Clear();
  _has_bits_[0] &= ~0x00000001u;
}
void Imu::clear_linear_acceleration() {
  if (linear_acceleration_ != nullptr) linear_acceleration_->Clear();
  _has_bits_[0] &= ~0x00000002u;
}
void Imu::clear_angular_velocity() {
  if (angular_velocity_ != nullptr) angular_velocity_->Clear();
  _has_bits_[0] &= ~0x00000004u;
}
Imu::Imu(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                         bool is_message_owned)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena, is_message_owned) {
  SharedCtor();
  if (!is_message_owned) {
    RegisterArenaDtor(arena);
  }
  // @@protoc_insertion_point(arena_constructor:apollo.drivers.gnss.Imu)
}
Imu::Imu(const Imu& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _has_bits_(from._has_bits_) {
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  if (from._internal_has_header()) {
    header_ = new ::apollo::common::Header(*from.header_);
  } else {
    header_ = nullptr;
  }
  if (from._internal_has_linear_acceleration()) {
    linear_acceleration_ = new ::apollo::common::Point3D(*from.linear_acceleration_);
  } else {
    linear_acceleration_ = nullptr;
  }
  if (from._internal_has_angular_velocity()) {
    angular_velocity_ = new ::apollo::common::Point3D(*from.angular_velocity_);
  } else {
    angular_velocity_ = nullptr;
  }
  ::memcpy(&measurement_time_, &from.measurement_time_,
    static_cast<size_t>(reinterpret_cast<char*>(&measurement_span_) -
    reinterpret_cast<char*>(&measurement_time_)) + sizeof(measurement_span_));
  // @@protoc_insertion_point(copy_constructor:apollo.drivers.gnss.Imu)
}

void Imu::SharedCtor() {
::memset(reinterpret_cast<char*>(this) + static_cast<size_t>(
    reinterpret_cast<char*>(&header_) - reinterpret_cast<char*>(this)),
    0, static_cast<size_t>(reinterpret_cast<char*>(&measurement_span_) -
    reinterpret_cast<char*>(&header_)) + sizeof(measurement_span_));
}

Imu::~Imu() {
  // @@protoc_insertion_point(destructor:apollo.drivers.gnss.Imu)
  if (GetArenaForAllocation() != nullptr) return;
  SharedDtor();
  _internal_metadata_.Delete<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

inline void Imu::SharedDtor() {
  GOOGLE_DCHECK(GetArenaForAllocation() == nullptr);
  if (this != internal_default_instance()) delete header_;
  if (this != internal_default_instance()) delete linear_acceleration_;
  if (this != internal_default_instance()) delete angular_velocity_;
}

void Imu::ArenaDtor(void* object) {
  Imu* _this = reinterpret_cast< Imu* >(object);
  (void)_this;
}
void Imu::RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena*) {
}
void Imu::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}

void Imu::Clear() {
// @@protoc_insertion_point(message_clear_start:apollo.drivers.gnss.Imu)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000007u) {
    if (cached_has_bits & 0x00000001u) {
      GOOGLE_DCHECK(header_ != nullptr);
      header_->Clear();
    }
    if (cached_has_bits & 0x00000002u) {
      GOOGLE_DCHECK(linear_acceleration_ != nullptr);
      linear_acceleration_->Clear();
    }
    if (cached_has_bits & 0x00000004u) {
      GOOGLE_DCHECK(angular_velocity_ != nullptr);
      angular_velocity_->Clear();
    }
  }
  if (cached_has_bits & 0x00000018u) {
    ::memset(&measurement_time_, 0, static_cast<size_t>(
        reinterpret_cast<char*>(&measurement_span_) -
        reinterpret_cast<char*>(&measurement_time_)) + sizeof(measurement_span_));
  }
  _has_bits_.Clear();
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* Imu::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  _Internal::HasBits has_bits{};
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    switch (tag >> 3) {
      // optional .apollo.common.Header header = 1;
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 10)) {
          ptr = ctx->ParseMessage(_internal_mutable_header(), ptr);
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // optional double measurement_time = 2;
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 17)) {
          _Internal::set_has_measurement_time(&has_bits);
          measurement_time_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<double>(ptr);
          ptr += sizeof(double);
        } else
          goto handle_unusual;
        continue;
      // optional float measurement_span = 3 [default = 0];
      case 3:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 29)) {
          _Internal::set_has_measurement_span(&has_bits);
          measurement_span_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<float>(ptr);
          ptr += sizeof(float);
        } else
          goto handle_unusual;
        continue;
      // optional .apollo.common.Point3D linear_acceleration = 4;
      case 4:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 34)) {
          ptr = ctx->ParseMessage(_internal_mutable_linear_acceleration(), ptr);
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // optional .apollo.common.Point3D angular_velocity = 5;
      case 5:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 42)) {
          ptr = ctx->ParseMessage(_internal_mutable_angular_velocity(), ptr);
          CHK_(ptr);
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
  _has_bits_.Or(has_bits);
  return ptr;
failure:
  ptr = nullptr;
  goto message_done;
#undef CHK_
}

::PROTOBUF_NAMESPACE_ID::uint8* Imu::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:apollo.drivers.gnss.Imu)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional .apollo.common.Header header = 1;
  if (cached_has_bits & 0x00000001u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(
        1, _Internal::header(this), target, stream);
  }

  // optional double measurement_time = 2;
  if (cached_has_bits & 0x00000008u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteDoubleToArray(2, this->_internal_measurement_time(), target);
  }

  // optional float measurement_span = 3 [default = 0];
  if (cached_has_bits & 0x00000010u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteFloatToArray(3, this->_internal_measurement_span(), target);
  }

  // optional .apollo.common.Point3D linear_acceleration = 4;
  if (cached_has_bits & 0x00000002u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(
        4, _Internal::linear_acceleration(this), target, stream);
  }

  // optional .apollo.common.Point3D angular_velocity = 5;
  if (cached_has_bits & 0x00000004u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(
        5, _Internal::angular_velocity(this), target, stream);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:apollo.drivers.gnss.Imu)
  return target;
}

size_t Imu::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:apollo.drivers.gnss.Imu)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x0000001fu) {
    // optional .apollo.common.Header header = 1;
    if (cached_has_bits & 0x00000001u) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
          *header_);
    }

    // optional .apollo.common.Point3D linear_acceleration = 4;
    if (cached_has_bits & 0x00000002u) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
          *linear_acceleration_);
    }

    // optional .apollo.common.Point3D angular_velocity = 5;
    if (cached_has_bits & 0x00000004u) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
          *angular_velocity_);
    }

    // optional double measurement_time = 2;
    if (cached_has_bits & 0x00000008u) {
      total_size += 1 + 8;
    }

    // optional float measurement_span = 3 [default = 0];
    if (cached_has_bits & 0x00000010u) {
      total_size += 1 + 4;
    }

  }
  return MaybeComputeUnknownFieldsSize(total_size, &_cached_size_);
}

const ::PROTOBUF_NAMESPACE_ID::Message::ClassData Imu::_class_data_ = {
    ::PROTOBUF_NAMESPACE_ID::Message::CopyWithSizeCheck,
    Imu::MergeImpl
};
const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*Imu::GetClassData() const { return &_class_data_; }

void Imu::MergeImpl(::PROTOBUF_NAMESPACE_ID::Message* to,
                      const ::PROTOBUF_NAMESPACE_ID::Message& from) {
  static_cast<Imu *>(to)->MergeFrom(
      static_cast<const Imu &>(from));
}


void Imu::MergeFrom(const Imu& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:apollo.drivers.gnss.Imu)
  GOOGLE_DCHECK_NE(&from, this);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 0x0000001fu) {
    if (cached_has_bits & 0x00000001u) {
      _internal_mutable_header()->::apollo::common::Header::MergeFrom(from._internal_header());
    }
    if (cached_has_bits & 0x00000002u) {
      _internal_mutable_linear_acceleration()->::apollo::common::Point3D::MergeFrom(from._internal_linear_acceleration());
    }
    if (cached_has_bits & 0x00000004u) {
      _internal_mutable_angular_velocity()->::apollo::common::Point3D::MergeFrom(from._internal_angular_velocity());
    }
    if (cached_has_bits & 0x00000008u) {
      measurement_time_ = from.measurement_time_;
    }
    if (cached_has_bits & 0x00000010u) {
      measurement_span_ = from.measurement_span_;
    }
    _has_bits_[0] |= cached_has_bits;
  }
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
}

void Imu::CopyFrom(const Imu& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:apollo.drivers.gnss.Imu)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool Imu::IsInitialized() const {
  return true;
}

void Imu::InternalSwap(Imu* other) {
  using std::swap;
  _internal_metadata_.InternalSwap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  ::PROTOBUF_NAMESPACE_ID::internal::memswap<
      PROTOBUF_FIELD_OFFSET(Imu, measurement_span_)
      + sizeof(Imu::measurement_span_)
      - PROTOBUF_FIELD_OFFSET(Imu, header_)>(
          reinterpret_cast<char*>(&header_),
          reinterpret_cast<char*>(&other->header_));
}

::PROTOBUF_NAMESPACE_ID::Metadata Imu::GetMetadata() const {
  return ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(
      &descriptor_table_modules_2fdrivers_2fgnss_2fproto_2fimu_2eproto_getter, &descriptor_table_modules_2fdrivers_2fgnss_2fproto_2fimu_2eproto_once,
      file_level_metadata_modules_2fdrivers_2fgnss_2fproto_2fimu_2eproto[0]);
}

// @@protoc_insertion_point(namespace_scope)
}  // namespace gnss
}  // namespace drivers
}  // namespace apollo
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::apollo::drivers::gnss::Imu* Arena::CreateMaybeMessage< ::apollo::drivers::gnss::Imu >(Arena* arena) {
  return Arena::CreateMessageInternal< ::apollo::drivers::gnss::Imu >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>