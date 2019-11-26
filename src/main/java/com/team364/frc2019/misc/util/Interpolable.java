package com.team364.frc2019.misc.util;

public interface Interpolable<T> {
    T interpolate(T other, double t);
}
