# Перша ціль = default. `make` без аргументів покаже help.
.PHONY: help build format lint test quality clean

help:
	@echo "Available targets:"
	@echo "  make build    — cmake configure + build (debug preset)"
	@echo "  make format   — clang-format -i for all C++ files"
	@echo "  make lint     — clang-tidy for all C++ files"
	@echo "  make test     — ctest for all unit-tests"
	@echo "  make quality  — format + lint + test (run before PR)"
	@echo "  make clean    — remove build folder"

build:
	cmake --preset debug
	cmake --build --preset debug

format:
	find homework_06 -type f \( -name '*.cpp' -o -name '*.hpp' \) -exec clang-format -i {} +
	cmake-format -i homework_06/CMakeLists.txt

lint: build
	clang-tidy -p build/debug homework_06/src/*.cpp homework_06/tests/*.cpp

test: build
	ctest --test-dir build/debug --output-on-failure

quality: format lint test

clean:
	rm -rf build