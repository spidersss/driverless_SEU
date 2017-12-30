include(functions)
find_package(Protobuf REQUIRED)

google_proto_library(lidar_frame
	SRCS
	./lidar_frame.proto
	)

