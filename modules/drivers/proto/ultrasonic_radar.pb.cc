// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: modules/drivers/proto/ultrasonic_radar.proto

#include "modules/drivers/proto/ultrasonic_radar.pb.h"

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
constexpr Ultrasonic::Ultrasonic(
  ::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized)
  : ranges_()
  , header_(nullptr){}
struct UltrasonicDefaultTypeInternal {
  constexpr UltrasonicDefaultTypeInternal()
    : _instance(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized{}) {}
  ~UltrasonicDefaultTypeInternal() {}
  union {
    Ultrasonic _instance;
  };
};
PROTOBUF_ATTRIBUTE_NO_DESTROY PROTOBUF_CONSTINIT UltrasonicDefaultTypeInternal _Ultrasonic_default_instance_;
}  // namespace drivers
}  // namespace apollo
static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_modules_2fdrivers_2fproto_2fultrasonic_5fradar_2eproto[1];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_modules_2fdrivers_2fproto_2fultrasonic_5fradar_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_modules_2fdrivers_2fproto_2fultrasonic_5fradar_2eproto = nullptr;

const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_modules_2fdrivers_2fproto_2fultrasonic_5fradar_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  PROTOBUF_FIELD_OFFSET(::apollo::drivers::Ultrasonic, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::apollo::drivers::Ultrasonic, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  ~0u,  // no _inlined_string_donated_
  PROTOBUF_FIELD_OFFSET(::apollo::drivers::Ultrasonic, header_),
  PROTOBUF_FIELD_OFFSET(::apollo::drivers::Ultrasonic, ranges_),
  0,
  ~0u,
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 8, -1, sizeof(::apollo::drivers::Ultrasonic)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::apollo::drivers::_Ultrasonic_default_instance_),
};

const char descriptor_table_protodef_modules_2fdrivers_2fproto_2fultrasonic_5fradar_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n,modules/drivers/proto/ultrasonic_radar"
  ".proto\022\016apollo.drivers\032!modules/common/p"
  "roto/header.proto\"C\n\nUltrasonic\022%\n\006heade"
  "r\030\001 \001(\0132\025.apollo.common.Header\022\016\n\006ranges"
  "\030\002 \003(\002"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_modules_2fdrivers_2fproto_2fultrasonic_5fradar_2eproto_deps[1] = {
  &::descriptor_table_modules_2fcommon_2fproto_2fheader_2eproto,
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_modules_2fdrivers_2fproto_2fultrasonic_5fradar_2eproto_once;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_modules_2fdrivers_2fproto_2fultrasonic_5fradar_2eproto = {
  false, false, 166, descriptor_table_protodef_modules_2fdrivers_2fproto_2fultrasonic_5fradar_2eproto, "modules/drivers/proto/ultrasonic_radar.proto", 
  &descriptor_table_modules_2fdrivers_2fproto_2fultrasonic_5fradar_2eproto_once, descriptor_table_modules_2fdrivers_2fproto_2fultrasonic_5fradar_2eproto_deps, 1, 1,
  schemas, file_default_instances, TableStruct_modules_2fdrivers_2fproto_2fultrasonic_5fradar_2eproto::offsets,
  file_level_metadata_modules_2fdrivers_2fproto_2fultrasonic_5fradar_2eproto, file_level_enum_descriptors_modules_2fdrivers_2fproto_2fultrasonic_5fradar_2eproto, file_level_service_descriptors_modules_2fdrivers_2fproto_2fultrasonic_5fradar_2eproto,
};
PROTOBUF_ATTRIBUTE_WEAK const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable* descriptor_table_modules_2fdrivers_2fproto_2fultrasonic_5fradar_2eproto_getter() {
  return &descriptor_table_modules_2fdrivers_2fproto_2fultrasonic_5fradar_2eproto;
}

// Force running AddDescriptors() at dynamic initialization time.
PROTOBUF_ATTRIBUTE_INIT_PRIORITY static ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptorsRunner dynamic_init_dummy_modules_2fdrivers_2fproto_2fultrasonic_5fradar_2eproto(&descriptor_table_modules_2fdrivers_2fproto_2fultrasonic_5fradar_2eproto);
namespace apollo {
namespace drivers {

// ===================================================================

class Ultrasonic::_Internal {
 public:
  using HasBits = decltype(std::declval<Ultrasonic>()._has_bits_);
  static const ::apollo::common::Header& header(const Ultrasonic* msg);
  static void set_has_header(HasBits* has_bits) {
    (*has_bits)[0] |= 1u;
  }
};

const ::apollo::common::Header&
Ultrasonic::_Internal::header(const Ultrasonic* msg) {
  return *msg->header_;
}
void Ultrasonic::clear_header() {
  if (header_ != nullptr) header_->Clear();
  _has_bits_[0] &= ~0x00000001u;
}
Ultrasonic::Ultrasonic(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                         bool is_message_owned)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena, is_message_owned),
  ranges_(arena) {
  SharedCtor();
  if (!is_message_owned) {
    RegisterArenaDtor(arena);
  }
  // @@protoc_insertion_point(arena_constructor:apollo.drivers.Ultrasonic)
}
Ultrasonic::Ultrasonic(const Ultrasonic& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _has_bits_(from._has_bits_),
      ranges_(from.ranges_) {
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  if (from._internal_has_header()) {
    header_ = new ::apollo::common::Header(*from.header_);
  } else {
    header_ = nullptr;
  }
  // @@protoc_insertion_point(copy_constructor:apollo.drivers.Ultrasonic)
}

void Ultrasonic::SharedCtor() {
header_ = nullptr;
}

Ultrasonic::~Ultrasonic() {
  // @@protoc_insertion_point(destructor:apollo.drivers.Ultrasonic)
  if (GetArenaForAllocation() != nullptr) return;
  SharedDtor();
  _internal_metadata_.Delete<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

inline void Ultrasonic::SharedDtor() {
  GOOGLE_DCHECK(GetArenaForAllocation() == nullptr);
  if (this != internal_default_instance()) delete header_;
}

void Ultrasonic::ArenaDtor(void* object) {
  Ultrasonic* _this = reinterpret_cast< Ultrasonic* >(object);
  (void)_this;
}
void Ultrasonic::RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena*) {
}
void Ultrasonic::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}

void Ultrasonic::Clear() {
// @@protoc_insertion_point(message_clear_start:apollo.drivers.Ultrasonic)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  ranges_.Clear();
  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000001u) {
    GOOGLE_DCHECK(header_ != nullptr);
    header_->Clear();
  }
  _has_bits_.Clear();
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* Ultrasonic::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
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
      // repeated float ranges = 2;
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 21)) {
          ptr -= 1;
          do {
            ptr += 1;
            _internal_add_ranges(::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<float>(ptr));
            ptr += sizeof(float);
            if (!ctx->DataAvailable(ptr)) break;
          } while (::PROTOBUF_NAMESPACE_ID::internal::ExpectTag<21>(ptr));
        } else if (static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 18) {
          ptr = ::PROTOBUF_NAMESPACE_ID::internal::PackedFloatParser(_internal_mutable_ranges(), ptr, ctx);
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

::PROTOBUF_NAMESPACE_ID::uint8* Ultrasonic::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:apollo.drivers.Ultrasonic)
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

  // repeated float ranges = 2;
  for (int i = 0, n = this->_internal_ranges_size(); i < n; i++) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteFloatToArray(2, this->_internal_ranges(i), target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:apollo.drivers.Ultrasonic)
  return target;
}

size_t Ultrasonic::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:apollo.drivers.Ultrasonic)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // repeated float ranges = 2;
  {
    unsigned int count = static_cast<unsigned int>(this->_internal_ranges_size());
    size_t data_size = 4UL * count;
    total_size += 1 *
                  ::PROTOBUF_NAMESPACE_ID::internal::FromIntSize(this->_internal_ranges_size());
    total_size += data_size;
  }

  // optional .apollo.common.Header header = 1;
  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000001u) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
        *header_);
  }

  return MaybeComputeUnknownFieldsSize(total_size, &_cached_size_);
}

const ::PROTOBUF_NAMESPACE_ID::Message::ClassData Ultrasonic::_class_data_ = {
    ::PROTOBUF_NAMESPACE_ID::Message::CopyWithSizeCheck,
    Ultrasonic::MergeImpl
};
const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*Ultrasonic::GetClassData() const { return &_class_data_; }

void Ultrasonic::MergeImpl(::PROTOBUF_NAMESPACE_ID::Message* to,
                      const ::PROTOBUF_NAMESPACE_ID::Message& from) {
  static_cast<Ultrasonic *>(to)->MergeFrom(
      static_cast<const Ultrasonic &>(from));
}


void Ultrasonic::MergeFrom(const Ultrasonic& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:apollo.drivers.Ultrasonic)
  GOOGLE_DCHECK_NE(&from, this);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  ranges_.MergeFrom(from.ranges_);
  if (from._internal_has_header()) {
    _internal_mutable_header()->::apollo::common::Header::MergeFrom(from._internal_header());
  }
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
}

void Ultrasonic::CopyFrom(const Ultrasonic& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:apollo.drivers.Ultrasonic)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool Ultrasonic::IsInitialized() const {
  return true;
}

void Ultrasonic::InternalSwap(Ultrasonic* other) {
  using std::swap;
  _internal_metadata_.InternalSwap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  ranges_.InternalSwap(&other->ranges_);
  swap(header_, other->header_);
}

::PROTOBUF_NAMESPACE_ID::Metadata Ultrasonic::GetMetadata() const {
  return ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(
      &descriptor_table_modules_2fdrivers_2fproto_2fultrasonic_5fradar_2eproto_getter, &descriptor_table_modules_2fdrivers_2fproto_2fultrasonic_5fradar_2eproto_once,
      file_level_metadata_modules_2fdrivers_2fproto_2fultrasonic_5fradar_2eproto[0]);
}

// @@protoc_insertion_point(namespace_scope)
}  // namespace drivers
}  // namespace apollo
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::apollo::drivers::Ultrasonic* Arena::CreateMaybeMessage< ::apollo::drivers::Ultrasonic >(Arena* arena) {
  return Arena::CreateMessageInternal< ::apollo::drivers::Ultrasonic >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
