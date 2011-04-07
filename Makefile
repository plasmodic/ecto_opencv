EXTRA_CMAKE_FLAGS= -DROSIFY=ON -Decto_DIR=$(shell rospack find ecto)/build
include $(shell rospack find mk)/cmake.mk
