#if __INTELLISENSE__
#undef __ARM_NEON
#undef __ARM_NEON__
#endif
// Fix from https://github.com/microsoft/vscode-cpptools/issues/7413#:~:text=containing%20the%20fix-,%23if%20__INTELLISENSE__%0A%23undef%20__ARM_NEON%0A%23undef%20__ARM_NEON__%0A%23endif,-2.%20Add%20it