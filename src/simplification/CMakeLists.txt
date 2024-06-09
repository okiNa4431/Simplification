set(BUILD_TARGET "Simplification")

#実行ファイル生成
add_executable(${BUILD_TARGET})
target_sources(${BUILD_TARGET} PUBLIC "main.cpp" "Simplification.h" "Simplification.cpp" ${COMMON_HEADERS})

#Eigenの設定
target_include_directories(${BUILD_TARGET} PUBLIC /Libraries/eigen-3.4.0)