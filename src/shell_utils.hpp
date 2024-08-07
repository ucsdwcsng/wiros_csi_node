#pragma once
#include <cstdio>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <array>

void sh_exec(std::string cmd);
std::string sh_exec_block(std::string cmd);
