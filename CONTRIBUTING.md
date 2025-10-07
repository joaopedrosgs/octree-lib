# Contributing to Octree Library

Thank you for your interest in contributing to the Octree Library! This document provides guidelines and instructions for contributing.

## Code of Conduct

Be respectful, constructive, and professional in all interactions.

## How to Contribute

### Reporting Bugs

1. Check if the bug has already been reported in [Issues](https://github.com/joaopedrosgs/octree-lib/issues)
2. If not, create a new issue with:
   - Clear, descriptive title
   - Steps to reproduce
   - Expected vs actual behavior
   - Your environment (OS, compiler, CMake version)
   - Code snippet demonstrating the issue (if applicable)

### Suggesting Features

1. Check existing issues and discussions
2. Create a new issue with:
   - Clear use case description
   - Proposed API/implementation (if applicable)
   - Why this feature would be valuable

### Pull Requests

#### Before You Start

1. Fork the repository
2. Create a feature branch from `main`
3. Ensure your development environment is set up:
   ```bash
   git clone https://github.com/YOUR_USERNAME/octree-lib.git
   cd octree-lib
   mkdir build && cd build
   cmake ..
   cmake --build .
   ctest
   ```

#### Development Guidelines

**Code Style:**
- Follow C++17 standards
- Use the provided `.clang-format` configuration
- Keep header-only design (all code in `include/octree/`)
- Use meaningful variable and function names
- Add comments for complex algorithms

**Testing:**
- Add tests for new features in `tests/`
- Ensure all existing tests pass
- Aim for high test coverage
- Use Google Test framework

**Benchmarks:**
- Add benchmarks for performance-critical features
- Document performance characteristics
- Use Google Benchmark framework

**Examples:**
- Provide example usage for new features
- Keep examples simple and focused
- Add comments explaining the code

#### Submitting a Pull Request

1. **Update your fork:**
   ```bash
   git checkout main
   git pull upstream main
   git checkout your-feature-branch
   git rebase main
   ```

2. **Ensure quality:**
   ```bash
   # Build and test
   cd build
   cmake --build . --config Release
   ctest -C Release --output-on-failure

   # Format code (optional, CI will check)
   find ../include ../tests ../examples -name "*.hpp" -o -name "*.cpp" | xargs clang-format -i
   ```

3. **Commit changes:**
   ```bash
   git add .
   git commit -m "Brief description of changes

   Detailed explanation of what changed and why.
   Include any breaking changes or migration notes.

   Fixes #123"
   ```

4. **Push and create PR:**
   ```bash
   git push origin your-feature-branch
   ```
   Then open a pull request on GitHub.

5. **PR Description should include:**
   - What problem does this solve?
   - How does it solve it?
   - Any breaking changes?
   - Test results
   - Screenshots/output (if applicable)

#### PR Review Process

- Maintainers will review your PR
- CI must pass (builds on Linux/macOS/Windows, all tests pass)
- Address review comments
- Once approved, maintainers will merge

## Project Structure

```
octree-lib/
├── include/octree/     # Header-only library (core code)
├── tests/              # Unit tests (Google Test)
├── benchmarks/         # Performance benchmarks (Google Benchmark)
├── examples/           # Usage examples
├── cmake/              # CMake configuration files
└── .github/workflows/  # CI/CD configuration
```

## Coding Standards

### Header-Only Library

All implementation must be in headers (`include/octree/`). This allows:
- Easy integration
- Template flexibility
- No linking required

### Templates

- Use templates for flexibility (coordinate types, data types)
- Provide clear template parameter documentation
- Consider compile-time performance

### Performance

- Optimize for common cases
- Document time/space complexity
- Add benchmarks for new features
- Avoid unnecessary allocations

### Memory Safety

- Use smart pointers (`std::unique_ptr`, `std::shared_ptr`)
- Avoid raw `new`/`delete`
- Use RAII principles
- No memory leaks (test with sanitizers)

### Error Handling

- Use exceptions for exceptional cases
- Validate input parameters
- Document exceptions in function comments
- Provide clear error messages

## Testing

### Unit Tests

```cpp
TEST(CategoryTest, FeatureName) {
    // Arrange
    Octree<double, int> tree(boundary);

    // Act
    bool result = tree.insert(point, data);

    // Assert
    EXPECT_TRUE(result);
}
```

### Benchmarks

```cpp
static void BM_FeatureName(benchmark::State& state) {
    // Setup
    Octree<double, int> tree(boundary);

    // Benchmark loop
    for (auto _ : state) {
        tree.someOperation();
    }

    // Report metrics
    state.counters["ItemsProcessed"] = ...;
}
BENCHMARK(BM_FeatureName)->Range(100, 100000);
```

## Documentation

- Update README.md for new features
- Add code examples
- Document API changes
- Update performance characteristics
- Add inline comments for complex logic

## License

By contributing, you agree that your contributions will be licensed under the MIT License.

## Questions?

- Open an issue for questions
- Tag with `question` label
- Be specific about what you need help with

## Recognition

Contributors will be acknowledged in the project. Thank you for making this library better! 🎉
