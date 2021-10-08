// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: modules/planning/proto/sl_boundary.proto

#include "modules/planning/proto/sl_boundary.pb.h"

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
namespace planning {
constexpr SLBoundary::SLBoundary(
  ::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized)
  : boundary_point_()
  , start_s_(0)
  , end_s_(0)
  , start_l_(0)
  , end_l_(0){}
struct SLBoundaryDefaultTypeInternal {
  constexpr SLBoundaryDefaultTypeInternal()
    : _instance(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized{}) {}
  ~SLBoundaryDefaultTypeInternal() {}
  union {
    SLBoundary _instance;
  };
};
PROTOBUF_ATTRIBUTE_NO_DESTROY PROTOBUF_CONSTINIT SLBoundaryDefaultTypeInternal _SLBoundary_default_instance_;
}  // namespace planning
}  // namespace apollo
static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_modules_2fplanning_2fproto_2fsl_5fboundary_2eproto[1];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_modules_2fplanning_2fproto_2fsl_5fboundary_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_modules_2fplanning_2fproto_2fsl_5fboundary_2eproto = nullptr;

const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_modules_2fplanning_2fproto_2fsl_5fboundary_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  PROTOBUF_FIELD_OFFSET(::apollo::planning::SLBoundary, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::apollo::planning::SLBoundary, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  ~0u,  // no _inlined_string_donated_
  PROTOBUF_FIELD_OFFSET(::apollo::planning::SLBoundary, start_s_),
  PROTOBUF_FIELD_OFFSET(::apollo::planning::SLBoundary, end_s_),
  PROTOBUF_FIELD_OFFSET(::apollo::planning::SLBoundary, start_l_),
  PROTOBUF_FIELD_OFFSET(::apollo::planning::SLBoundary, end_l_),
  PROTOBUF_FIELD_OFFSET(::apollo::planning::SLBoundary, boundary_point_),
  0,
  1,
  2,
  3,
  ~0u,
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 11, -1, sizeof(::apollo::planning::SLBoundary)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::apollo::planning::_SLBoundary_default_instance_),
};

const char descriptor_table_protodef_modules_2fplanning_2fproto_2fsl_5fboundary_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n(modules/planning/proto/sl_boundary.pro"
  "to\022\017apollo.planning\032$modules/common/prot"
  "o/pnc_point.proto\"|\n\nSLBoundary\022\017\n\007start"
  "_s\030\001 \001(\001\022\r\n\005end_s\030\002 \001(\001\022\017\n\007start_l\030\003 \001(\001"
  "\022\r\n\005end_l\030\004 \001(\001\022.\n\016boundary_point\030\005 \003(\0132"
  "\026.apollo.common.SLPoint"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_modules_2fplanning_2fproto_2fsl_5fboundary_2eproto_deps[1] = {
  &::descriptor_table_modules_2fcommon_2fproto_2fpnc_5fpoint_2eproto,
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_modules_2fplanning_2fproto_2fsl_5fboundary_2eproto_once;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_modules_2fplanning_2fproto_2fsl_5fboundary_2eproto = {
  false, false, 223, descriptor_table_protodef_modules_2fplanning_2fproto_2fsl_5fboundary_2eproto, "modules/planning/proto/sl_boundary.proto", 
  &descriptor_table_modules_2fplanning_2fproto_2fsl_5fboundary_2eproto_once, descriptor_table_modules_2fplanning_2fproto_2fsl_5fboundary_2eproto_deps, 1, 1,
  schemas, file_default_instances, TableStruct_modules_2fplanning_2fproto_2fsl_5fboundary_2eproto::offsets,
  file_level_metadata_modules_2fplanning_2fproto_2fsl_5fboundary_2eproto, file_level_enum_descriptors_modules_2fplanning_2fproto_2fsl_5fboundary_2eproto, file_level_service_descriptors_modules_2fplanning_2fproto_2fsl_5fboundary_2eproto,
};
PROTOBUF_ATTRIBUTE_WEAK const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable* descriptor_table_modules_2fplanning_2fproto_2fsl_5fboundary_2eproto_getter() {
  return &descriptor_table_modules_2fplanning_2fproto_2fsl_5fboundary_2eproto;
}

// Force running AddDescriptors() at dynamic initialization time.
PROTOBUF_ATTRIBUTE_INIT_PRIORITY static ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptorsRunner dynamic_init_dummy_modules_2fplanning_2fproto_2fsl_5fboundary_2eproto(&descriptor_table_modules_2fplanning_2fproto_2fsl_5fboundary_2eproto);
namespace apollo {
namespace planning {

// ===================================================================

class SLBoundary::_Internal {
 public:
  using HasBits = decltype(std::declval<SLBoundary>()._has_bits_);
  static void set_has_start_s(HasBits* has_bits) {
    (*has_bits)[0] |= 1u;
  }
  static void set_has_end_s(HasBits* has_bits) {
    (*has_bits)[0] |= 2u;
  }
  static void set_has_start_l(HasBits* has_bits) {
    (*has_bits)[0] |= 4u;
  }
  static void set_has_end_l(HasBits* has_bits) {
    (*has_bits)[0] |= 8u;
  }
};

void SLBoundary::clear_boundary_point() {
  boundary_point_.Clear();
}
SLBoundary::SLBoundary(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                         bool is_message_owned)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena, is_message_owned),
  boundary_point_(arena) {
  SharedCtor();
  if (!is_message_owned) {
    RegisterArenaDtor(arena);
  }
  // @@protoc_insertion_point(arena_constructor:apollo.planning.SLBoundary)
}
SLBoundary::SLBoundary(const SLBoundary& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _has_bits_(from._has_bits_),
      boundary_point_(from.boundary_point_) {
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  ::memcpy(&start_s_, &from.start_s_,
    static_cast<size_t>(reinterpret_cast<char*>(&end_l_) -
    reinterpret_cast<char*>(&start_s_)) + sizeof(end_l_));
  // @@protoc_insertion_point(copy_constructor:apollo.planning.SLBoundary)
}

void SLBoundary::SharedCtor() {
::memset(reinterpret_cast<char*>(this) + static_cast<size_t>(
    reinterpret_cast<char*>(&start_s_) - reinterpret_cast<char*>(this)),
    0, static_cast<size_t>(reinterpret_cast<char*>(&end_l_) -
    reinterpret_cast<char*>(&start_s_)) + sizeof(end_l_));
}

SLBoundary::~SLBoundary() {
  // @@protoc_insertion_point(destructor:apollo.planning.SLBoundary)
  if (GetArenaForAllocation() != nullptr) return;
  SharedDtor();
  _internal_metadata_.Delete<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

inline void SLBoundary::SharedDtor() {
  GOOGLE_DCHECK(GetArenaForAllocation() == nullptr);
}

void SLBoundary::ArenaDtor(void* object) {
  SLBoundary* _this = reinterpret_cast< SLBoundary* >(object);
  (void)_this;
}
void SLBoundary::RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena*) {
}
void SLBoundary::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}

void SLBoundary::Clear() {
// @@protoc_insertion_point(message_clear_start:apollo.planning.SLBoundary)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  boundary_point_.Clear();
  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x0000000fu) {
    ::memset(&start_s_, 0, static_cast<size_t>(
        reinterpret_cast<char*>(&end_l_) -
        reinterpret_cast<char*>(&start_s_)) + sizeof(end_l_));
  }
  _has_bits_.Clear();
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* SLBoundary::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  _Internal::HasBits has_bits{};
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    switch (tag >> 3) {
      // optional double start_s = 1;
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 9)) {
          _Internal::set_has_start_s(&has_bits);
          start_s_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<double>(ptr);
          ptr += sizeof(double);
        } else
          goto handle_unusual;
        continue;
      // optional double end_s = 2;
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 17)) {
          _Internal::set_has_end_s(&has_bits);
          end_s_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<double>(ptr);
          ptr += sizeof(double);
        } else
          goto handle_unusual;
        continue;
      // optional double start_l = 3;
      case 3:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 25)) {
          _Internal::set_has_start_l(&has_bits);
          start_l_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<double>(ptr);
          ptr += sizeof(double);
        } else
          goto handle_unusual;
        continue;
      // optional double end_l = 4;
      case 4:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 33)) {
          _Internal::set_has_end_l(&has_bits);
          end_l_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<double>(ptr);
          ptr += sizeof(double);
        } else
          goto handle_unusual;
        continue;
      // repeated .apollo.common.SLPoint boundary_point = 5;
      case 5:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 42)) {
          ptr -= 1;
          do {
            ptr += 1;
            ptr = ctx->ParseMessage(_internal_add_boundary_point(), ptr);
            CHK_(ptr);
            if (!ctx->DataAvailable(ptr)) break;
          } while (::PROTOBUF_NAMESPACE_ID::internal::ExpectTag<42>(ptr));
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

::PROTOBUF_NAMESPACE_ID::uint8* SLBoundary::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:apollo.planning.SLBoundary)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional double start_s = 1;
  if (cached_has_bits & 0x00000001u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteDoubleToArray(1, this->_internal_start_s(), target);
  }

  // optional double end_s = 2;
  if (cached_has_bits & 0x00000002u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteDoubleToArray(2, this->_internal_end_s(), target);
  }

  // optional double start_l = 3;
  if (cached_has_bits & 0x00000004u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteDoubleToArray(3, this->_internal_start_l(), target);
  }

  // optional double end_l = 4;
  if (cached_has_bits & 0x00000008u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteDoubleToArray(4, this->_internal_end_l(), target);
  }

  // repeated .apollo.common.SLPoint boundary_point = 5;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->_internal_boundary_point_size()); i < n; i++) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(5, this->_internal_boundary_point(i), target, stream);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:apollo.planning.SLBoundary)
  return target;
}

size_t SLBoundary::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:apollo.planning.SLBoundary)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // repeated .apollo.common.SLPoint boundary_point = 5;
  total_size += 1UL * this->_internal_boundary_point_size();
  for (const auto& msg : this->boundary_point_) {
    total_size +=
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(msg);
  }

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x0000000fu) {
    // optional double start_s = 1;
    if (cached_has_bits & 0x00000001u) {
      total_size += 1 + 8;
    }

    // optional double end_s = 2;
    if (cached_has_bits & 0x00000002u) {
      total_size += 1 + 8;
    }

    // optional double start_l = 3;
    if (cached_has_bits & 0x00000004u) {
      total_size += 1 + 8;
    }

    // optional double end_l = 4;
    if (cached_has_bits & 0x00000008u) {
      total_size += 1 + 8;
    }

  }
  return MaybeComputeUnknownFieldsSize(total_size, &_cached_size_);
}

const ::PROTOBUF_NAMESPACE_ID::Message::ClassData SLBoundary::_class_data_ = {
    ::PROTOBUF_NAMESPACE_ID::Message::CopyWithSizeCheck,
    SLBoundary::MergeImpl
};
const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*SLBoundary::GetClassData() const { return &_class_data_; }

void SLBoundary::MergeImpl(::PROTOBUF_NAMESPACE_ID::Message* to,
                      const ::PROTOBUF_NAMESPACE_ID::Message& from) {
  static_cast<SLBoundary *>(to)->MergeFrom(
      static_cast<const SLBoundary &>(from));
}


void SLBoundary::MergeFrom(const SLBoundary& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:apollo.planning.SLBoundary)
  GOOGLE_DCHECK_NE(&from, this);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  boundary_point_.MergeFrom(from.boundary_point_);
  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 0x0000000fu) {
    if (cached_has_bits & 0x00000001u) {
      start_s_ = from.start_s_;
    }
    if (cached_has_bits & 0x00000002u) {
      end_s_ = from.end_s_;
    }
    if (cached_has_bits & 0x00000004u) {
      start_l_ = from.start_l_;
    }
    if (cached_has_bits & 0x00000008u) {
      end_l_ = from.end_l_;
    }
    _has_bits_[0] |= cached_has_bits;
  }
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
}

void SLBoundary::CopyFrom(const SLBoundary& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:apollo.planning.SLBoundary)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool SLBoundary::IsInitialized() const {
  return true;
}

void SLBoundary::InternalSwap(SLBoundary* other) {
  using std::swap;
  _internal_metadata_.InternalSwap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  boundary_point_.InternalSwap(&other->boundary_point_);
  ::PROTOBUF_NAMESPACE_ID::internal::memswap<
      PROTOBUF_FIELD_OFFSET(SLBoundary, end_l_)
      + sizeof(SLBoundary::end_l_)
      - PROTOBUF_FIELD_OFFSET(SLBoundary, start_s_)>(
          reinterpret_cast<char*>(&start_s_),
          reinterpret_cast<char*>(&other->start_s_));
}

::PROTOBUF_NAMESPACE_ID::Metadata SLBoundary::GetMetadata() const {
  return ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(
      &descriptor_table_modules_2fplanning_2fproto_2fsl_5fboundary_2eproto_getter, &descriptor_table_modules_2fplanning_2fproto_2fsl_5fboundary_2eproto_once,
      file_level_metadata_modules_2fplanning_2fproto_2fsl_5fboundary_2eproto[0]);
}

// @@protoc_insertion_point(namespace_scope)
}  // namespace planning
}  // namespace apollo
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::apollo::planning::SLBoundary* Arena::CreateMaybeMessage< ::apollo::planning::SLBoundary >(Arena* arena) {
  return Arena::CreateMessageInternal< ::apollo::planning::SLBoundary >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>