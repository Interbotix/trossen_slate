BUILD_DIR    := build
INSTALL_DIR  := install
NPROC        := $(shell nproc 2>/dev/null || echo 4)

.PHONY: build install clean demo python help
build:
	@cmake -S . -B $(BUILD_DIR) -DCMAKE_INSTALL_PREFIX=$(INSTALL_DIR)
	@cmake --build $(BUILD_DIR) -j$(NPROC)

install: build
	@cmake --install $(BUILD_DIR)

clean:
	@rm -rf $(BUILD_DIR) $(INSTALL_DIR)

demo:
	@cmake -S . -B $(BUILD_DIR) -DCMAKE_INSTALL_PREFIX=$(INSTALL_DIR) -DBUILD_DEMOS=ON
	@cmake --build $(BUILD_DIR) -j$(NPROC)
	@./$(BUILD_DIR)/basic_demo

python:
	@pip install .

help:
	@echo "trossen_slate build targets:"
	@echo ""
	@echo "  make build        Build the project"
	@echo "  make install      Build and install to ./install"
	@echo "  make clean        Remove build/ and install/ directories"
	@echo "  make demo         Build and run basic_demo"
	@echo "  make python       Build and install the Python package"
	@echo ""
	@echo "Variables:"
	@echo "  BUILD_DIR=...     Set build directory (default: build)"
	@echo "  INSTALL_DIR=...   Set install prefix (default: install)"
	@echo "  NPROC=...         Set number of parallel build jobs (default: number of CPU cores)"
