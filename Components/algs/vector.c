#pragma once
#include <string.h>
#include "basic_algs.h"
#include "vector.h"

static inline uint32_t calc_growth(uint32_t capacity) {
    return capacity + capacity / 2;
}

void _vector_init(vector_t *v, uint32_t cell_size, uint32_t capacity) {
    v->pool = malloc(cell_size * capacity);
    v->capacity = capacity;
    v->cell_size = cell_size;
    v->size = 0;
    v->front = 0;
    v->rear = 0;
}

void _vector_deinit(vector_t *v) {
    free(v->pool);
}

vector_t *_vector_new(uint32_t cell_size, uint32_t capacity) {
    vector_t *v = malloc(sizeof(vector_t));
    _vector_init(v, cell_size, capacity);
    return v;
}

void _vector_delete(vector_t *v) {
    _vector_deinit(v);
    free(v);
}

void _vector_clear(vector_t *v) {
    v->size = 0;
    v->front = 0;
    v->rear = 0;
}

static void _vector_grow_capacity(vector_t *v, uint32_t new_capacity) {
    if (new_capacity <= v->capacity) return;
    void *new_pool = malloc(v->cell_size * new_capacity);
    for(int i=0;i<v->size;i++){
        memcpy((uint8_t *)new_pool + i * v->cell_size, (uint8_t *)v->pool + ((v->front + i) % v->capacity) * v->cell_size, v->cell_size);
    }
    free(v->pool);
    v->pool = new_pool;
    v->front = 0;
    v->rear = v->size;
    v->capacity = new_capacity;

    // v->pool = realloc(v->pool, v->cell_size * new_capacity);
    // if(v->rear < v->front) {    // 存在回环
    //     uint32_t need_move_count = v->rear;
    //     uint32_t moved_count = min(need_move_count, new_capacity - v->capacity);
    //     memcpy((uint8_t *)v->pool + v->capacity * v->cell_size, v->pool, moved_count * v->cell_size);
    //     if(need_move_count > moved_count) {
    //         memmove(v->pool, (uint8_t *)v->pool + moved_count * v->cell_size, (need_move_count - moved_count) * v->cell_size);
    //     }
    //     v->rear = (v->front + v->size) % new_capacity;
    // }
    // v->capacity = new_capacity;
}

void _vector_push_back(vector_t *v, void *data) {
    if (v->size == v->capacity) {
        _vector_grow_capacity(v, calc_growth(v->capacity));
    }
    memcpy((uint8_t *)v->pool + v->rear * v->cell_size, data, v->cell_size);
    v->rear = (v->rear + 1) % v->capacity;
    v->size++;
}

void _vector_pop_back(vector_t *v, void *data) {
    if (v->size == 0) return;
    v->rear = (v->rear - 1 + v->capacity) % v->capacity;
    if(data) memcpy(data, (uint8_t *)v->pool + v->rear * v->cell_size, v->cell_size);
    v->size--;
}

void _vector_push_front(vector_t *v, void *data) {
    if (v->size == v->capacity) {
        _vector_grow_capacity(v, calc_growth(v->capacity));
    }
    v->front = (v->front - 1 + v->capacity) % v->capacity;
    memcpy((uint8_t *)v->pool + v->front * v->cell_size, data, v->cell_size);
    v->size++;
}

void _vector_pop_front(vector_t *v, void *data) {
    if (v->size == 0) return;
    if(data) memcpy(data, (uint8_t *)v->pool + v->front * v->cell_size, v->cell_size);
    v->front = (v->front + 1) % v->capacity;
    v->size--;
}

void vector_reverse(vector_t *v) {
    int i = 0, j = v->size - 1;
    uint8_t tmp[v->cell_size];
    while (i < j) {
        uint8_t *a = (uint8_t *)v->pool + (v->front + i) % v->capacity * v->cell_size;
        uint8_t *b = (uint8_t *)v->pool + (v->front + j) % v->capacity * v->cell_size;
        memcpy(tmp, a, v->cell_size);
        memcpy(a, b, v->cell_size);
        memcpy(b, tmp, v->cell_size);
        i++;
        j--;
    }
}