#
# -*- Makefile -*-
#
# @file  	config.mk
# @brief 	Auto compile library 
# @date  	2011.03.27
# @author	Kei Sugawara
#
# @par History
# - 2011.03.27 Kei Sugawara
#	-# Initial Version
#

# DIR ------>> コンパイルを実行した絶対パス
DIR = $(shell pwd)

# BIN ----->>出力ファイル名
BIN = $(notdir $(DIR))

# SRC_DIR ------>> ソースファイルの相対パス
SRC_DIR = ./src/

# INC_DIR ----->> インクルードパス
INC_DIR +=  -I./include -I/usr/include

# LIBS ----->> 標準ライブラリ
LIBS += -L/usr/lib

# BUILD_DIR ----->>コンパイルに使用するファイルの相対パス
BUILD_DIR = ./build/

# BIN_DIR ----->>実行ファイルの相対パス
BIN_DIR = ./bin/

# オプション
CXXFLAGS = -O2 -Wall 

# SRC ----->>コンパイルするソースファイル名を取得
SRC = $(notdir $(shell echo ./src/*.cpp))

# 実行ファイル名を生成
BINARY = $(patsubst %,$(BIN_DIR)%,$(BIN))

# オブジェクトファイル名を生成
OBJS = $(patsubst %.cpp,$(BUILD_DIR)%.o,$(SRC))

# 依存関係ファイル名を生成
DEPENDS = $(patsubst %.cpp,$(BUILD_DIR)%.d,$(SRC))


all:$(BINARY)

$(BINARY): $(OBJS)
	@echo "Linking :" $(@F)
	@$(CXX)  $(CXXFLAGS) -o $@ $^ $(LIBS) $(INC_DIR)

$(OBJS): $(BUILD_DIR)%.o: $(SRC_DIR)%.cpp
	@echo "Compiling :" $(@F)
	@$(CXX)  $(CXXFLAGS) -o $@ -c $< $(INC_DIR)


# オブジェクト、依存関係、実行ファイルの削除
clean :
	@echo "All cleaned ..."
	@$(RM) *~ $(BINARY) $(OBJS) $(DEPENDS)

#------------------------------------------------------

# 依存関係用
$(DEPENDS) : $(BUILD_DIR)%.d: $(SRC_DIR)%.cpp
	@echo "Dependency :" $(@F)
	@set -e; $(CXX) -MM $(CXXFLAGS) $(INC_DIR) $< \
	 		| sed 's/\($*\)\.o[ :]*/\1.o build\/$(@F) : /g' \
			| sed 's/$*.o/build\/$*.o/g'> $@; \
			[ -s $@ ] || rm -f $@
# 生成された依存関係を読み込む(-はエラーを無視させるため)
-include $(DEPENDS)
