#!/bin/bash

SOLUTION_FILE=$1
INPUT_FILE=$2
OUTPUT_FILE=$3
TIMEOUT_MS=$4

# Extract LANG_TYPE from SOLUTION_FILE extension
case "$SOLUTION_FILE" in
    *.c)      LANG_TYPE=C      ;;
    *.cpp)    LANG_TYPE=CPP    ;;
    *.java)   LANG_TYPE=JAVA   ;;
    *.py)     LANG_TYPE=PYTHON ;;
    *)        echo "Unsupported solution file extension: $SOLUTION_FILE"
              exit 5
              ;;
esac

# 处理不同语言的扩展名移除
case $LANG_TYPE in
    C)      BASE_NAME=$(basename "$SOLUTION_FILE" .c)      ;;
    CPP)    BASE_NAME=$(basename "$SOLUTION_FILE" .cpp)    ;;
    JAVA)   BASE_NAME=$(basename "$SOLUTION_FILE" .java)   ;;
    PYTHON) BASE_NAME=$(basename "$SOLUTION_FILE" .py)     ;;
    *)      BASE_NAME=$(basename "$SOLUTION_FILE")         ;;
esac
SOLUTION_PATH=$(dirname "$SOLUTION_FILE")

if [ -z "$SOLUTION_FILE" ] || [ -z "$INPUT_FILE" ] || [ -z "$OUTPUT_FILE" ] || [ -z "$LANG_TYPE" ]; then
    echo "Usage: $0 <solution_file> <input_file> <output_file> [timeout_ms]"
    echo "Example: $0 ./demos/Solution.cpp ./data/sample.in ./data/sample.out 90000"
    echo "Supported languages: C, CPP, JAVA, PYTHON"
    echo "  timeout_ms: total interaction timeout in milliseconds"
    exit 5
fi

case $LANG_TYPE in
    C)
        gcc -std=c17 $SOLUTION_FILE -o $SOLUTION_PATH/$BASE_NAME -O2 -lm
        if [ $? -ne 0 ]; then
            echo "Compilation error: $SOLUTION_FILE compilation failed"
            exit 2
        fi
        if [ ! -f $SOLUTION_PATH/$BASE_NAME ]; then
            echo "File not found: $SOLUTION_PATH/$BASE_NAME file does not exist"
            exit 6
        fi
        ;;
    CPP)
        g++ -std=c++17 $SOLUTION_FILE -o $SOLUTION_PATH/$BASE_NAME -O2
        if [ $? -ne 0 ]; then
            echo "Compilation error: $SOLUTION_FILE compilation failed"
            exit 2
        fi
        if [ ! -f $SOLUTION_PATH/$BASE_NAME ]; then
            echo "File not found: $SOLUTION_PATH/$BASE_NAME file does not exist"
            exit 6
        fi
        ;;
    JAVA)
        javac -d $SOLUTION_PATH -implicit:none $SOLUTION_FILE
        if [ $? -ne 0 ]; then
            echo "Compilation error: $SOLUTION_FILE compilation failed"
            exit 2
        fi
        if [ ! -f "$SOLUTION_PATH/$BASE_NAME.class" ]; then
            echo "File not found: $SOLUTION_PATH/$BASE_NAME.class file does not exist"
            exit 6
        fi
        ;;
    PYTHON)
        ;;
    *)
        echo "Unsupported language: $LANG_TYPE"
        exit 5
        ;;
esac

echo "选手作品编译完成"

if [ ! -f ./Interactor ]; then
    echo "File not found: ./Interactor file does not exist"
    exit 6
fi

if [ ! -f ./Runner ]; then
    echo "File not found: ./Runner file does not exist"
    exit 6
fi

chmod +x Runner Interactor

echo "Begin running ..."
./Runner "$INPUT_FILE" "$OUTPUT_FILE" "$LANG_TYPE" "$SOLUTION_PATH" "$BASE_NAME" "$TIMEOUT_MS"

echo "Finish process"
exit 0
