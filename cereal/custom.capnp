using Cxx = import "./include/c++.capnp";
$Cxx.namespace("cereal");

@0xb526ba661d550a59;

# custom.capnp: a home for empty structs reserved for custom forks
# These structs are guaranteed to remain reserved and empty in mainline
# cereal, so use these if you want custom events in your fork.

# DO rename the structs
# DON'T change the identifier (e.g. @0x81c2f05a394cf4af)

# comma 비전 모델의 차선유지용 예측 주행경로 (modeld → udp_bridge).
# modeld가 매 프레임 계산하지만 버리던 plan/position 을 외부경로 융합에 쓰기 위해 노출.
# 좌표계: modelV2.position 과 동일 (x=forward[m], y 는 모델 native frame). udp_bridge 에서 내부규약으로 변환.
struct ModelLanePath @0x81c2f05a394cf4af {
  frameId @0 :UInt32;
  valid @1 :Bool;
  vEgo @2 :Float32;
  positionX @3 :List(Float32);      # T_IDXS 33pts, forward [m]
  positionY @4 :List(Float32);      # T_IDXS 33pts, lateral [m] (modelV2.position 규약)
  desiredCurvature @5 :Float32;     # 모델 자체 차선유지 곡률 (diag/fallback 용)
}

struct CustomReserved1 @0xaedffd8f31e7b55d {
}

struct CustomReserved2 @0xf35cc4560bbf6ec2 {
}

struct CustomReserved3 @0xda96579883444c35 {
}

struct CustomReserved4 @0x80ae746ee2596b11 {
}

struct CustomReserved5 @0xa5cd762cd951a455 {
}

struct CustomReserved6 @0xf98d843bfd7004a3 {
}

struct CustomReserved7 @0xb86e6369214c01c8 {
}

struct CustomReserved8 @0xf416ec09499d9d19 {
}

struct CustomReserved9 @0xa1680744031fdb2d {
}

struct CustomReserved10 @0xcb9fd56c7057593a {
}

struct CustomReserved11 @0xc2243c65e0340384 {
}

struct CustomReserved12 @0x9ccdc8676701b412 {
}

struct CustomReserved13 @0xcd96dafb67a082d0 {
}

struct CustomReserved14 @0xb057204d7deadf3f {
}

struct CustomReserved15 @0xbd443b539493bc68 {
}

struct CustomReserved16 @0xfc6241ed8877b611 {
}

struct CustomReserved17 @0xa30662f84033036c {
}

struct CustomReserved18 @0xc86a3d38d13eb3ef {
}

struct CustomReserved19 @0xa4f1eb3323f5f582 {
}
