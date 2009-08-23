/******************************************************************************
 * Copyright (c) 2009 Kevin Wolf                                              *
 *                                                                            *
 * Permission is hereby granted,  free of charge,  to any  person obtaining a *
 * copy of this software and associated documentation files (the "Software"), *
 * to deal in the Software without restriction,  including without limitation *
 * the rights to use,  copy, modify, merge, publish,  distribute, sublicense, *
 * and/or sell copies  of the  Software,  and to permit  persons to whom  the *
 * Software is furnished to do so, subject to the following conditions:       *
 *                                                                            *
 * The above copyright notice and this permission notice shall be included in *
 * all copies or substantial portions of the Software.                        *
 *                                                                            *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR *
 * IMPLIED, INCLUDING  BUT NOT  LIMITED TO THE WARRANTIES OF MERCHANTABILITY, *
 * FITNESS FOR A PARTICULAR  PURPOSE AND  NONINFRINGEMENT.  IN NO EVENT SHALL *
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER *
 * LIABILITY,  WHETHER IN AN ACTION OF CONTRACT,  TORT OR OTHERWISE,  ARISING *
 * FROM,  OUT OF  OR IN CONNECTION  WITH THE  SOFTWARE  OR THE  USE OR  OTHER *
 * DEALINGS IN THE SOFTWARE.                                                  *
 ******************************************************************************/

#include <stdlib.h>
#include <stdint.h>

#include "cdi/misc.h"
#include "mempool.h"

struct free_object {
    struct free_object* next;
};

struct mempool {
    size_t pool_size;
    size_t object_size;
    size_t num_objects;

    void* objects;
    void* phys_objects;

    struct free_object* first_free;
};

struct mempool* mempool_create(size_t pool_size, size_t object_size)
{
    struct mempool* pool = malloc(sizeof(*pool));
    uintptr_t cur;
    int i;
    struct free_object** p;

    pool->pool_size = pool_size;
    pool->object_size = object_size;
    pool->num_objects = pool_size / object_size;

    // Wir brauchen auf jeden Fall Platz fuer die Freispeicherliste
    if (object_size < sizeof(struct free_object)) {
        goto fail;
    }


    // Speicher reservieren
    // FIXME Das ist alles zusammenhaengend
    if (cdi_alloc_phys_mem(pool_size, &pool->objects, &pool->phys_objects)) {
        goto fail;
    }

    // Freie Objekte eintragen
    cur = (uintptr_t) pool->objects;
    p = &pool->first_free;
    for (i = 0; i < pool->num_objects; i++) {
        *p = (struct free_object*) cur;
        p = &((struct free_object*) cur)->next;
        cur += object_size;
    }
    *p = NULL;

    return pool;

fail:
    free(pool);
    return NULL;
}

int mempool_get(struct mempool* pool, void** obj, uintptr_t* phys_obj)
{
    struct free_object* free = pool->first_free;

    if (free == NULL) {
        return -1;
    }

    pool->first_free = free->next;

    *obj = free;
    *phys_obj = (uintptr_t) pool->phys_objects +
        (uintptr_t) free - (uintptr_t) pool->objects;

    return 0;
}

int mempool_put(struct mempool* pool, void* obj)
{
    struct free_object* free = obj;

    free->next = pool->first_free;
    pool->first_free = free;

    return 0;
}
