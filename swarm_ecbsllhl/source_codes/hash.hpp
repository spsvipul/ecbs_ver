#pragma once

template<typename T>
void hash_combine(size_t & seed, T const& v) {
	std::hash<T> primitive_type_hash;
	seed ^= primitive_type_hash(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
}

