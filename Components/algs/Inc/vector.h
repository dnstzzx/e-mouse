#pragma once
#include <stdint.h>
#include <stdbool.h>

// NOTICE: GNU C EXTENSION IS REQUIRED

#define VECTOR_DEFAULT_CAPACITY (16)

//  无回环： [front, rear) 
//  有回环： [front, capacity) + [0, rear)
typedef struct {
    void *pool;
    uint32_t cell_size;
    uint32_t size;
    uint32_t capacity;
    uint32_t front, rear;
} vector_t;

vector_t *_vector_new(uint32_t cell_size, uint32_t capacity);
void _vector_delete(vector_t *v);
void _vector_init(vector_t *v, uint32_t cell_size, uint32_t capacity);
void _vector_deinit(vector_t *v);
void _vector_clear(vector_t *v);
void _vector_push_front(vector_t *v, void *data);
void _vector_pop_front(vector_t *v, void *data);
void _vector_push_back(vector_t *v, void *data);
void _vector_pop_back(vector_t *v, void *data);
void vector_reverse(vector_t *v);

static inline uint32_t vector_size(vector_t *v){ return v->size; }
static inline bool vector_empty(vector_t *v){ return v->size == 0; }
#define vector_clear(v) _vector_clear(v)

#define vector_new_with_capacity(type, capacity) _vector_new(sizeof(type), capacity)
#define vector_new(type) vector_new_with_capacity(type, VECTOR_DEFAULT_CAPACITY)
#define vector_delete(v) _vector_delete(v)
#define vector_init_with_capacity(v, type, capacity) _vector_init(v, sizeof(type), capacity)
#define vector_init(v, type) vector_init_with_capacity(v, type, VECTOR_DEFAULT_CAPACITY)

#define vector_front(v, type) (*(type *)((uint8_t *)v->pool + v->front * v->cell_size))
#define vector_back(v, type) (*(type *)((uint8_t *)v->pool + (v->rear - 1 + v->capacity) % v->capacity * v->cell_size))
#define vector_at(v, type, index) (*(type *)((uint8_t *)v->pool + (v->front + index) % v->capacity * v->cell_size))

#define vector_push_back(v, data) _vector_push_back(v, (void *)&(data))
#define vector_pop_back_to(v, data) _vector_pop_back(v, (void *)&(data))
#define vector_pop_back(v, type) ({type rtn = vector_back(v, type) ;_vector_pop_back(v, NULL);rtn;})

#define vector_push_front(v, data) _vector_push_front(v, (void *)&(data))
#define vector_pop_front_to(v, data) _vector_pop_front(v, (void *)&(data))
#define vector_pop_front(v, type) ({type rtn = vector_front(v, type) ;_vector_pop_front(v, NULL);rtn;})



#define vector_for_each(v, type, element) \
    for(uint32_t i = 0; i < v->size; i++)  \
        for(;;) \
        for(type element = vector_at(v, type, i);;({break;}))
