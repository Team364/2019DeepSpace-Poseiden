package com.team364.misc.util;

public interface Interpolable<T> {
    T interpolate(T other, double t);
}
