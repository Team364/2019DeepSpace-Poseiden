package com.team364.frc2019.misc.util;

public interface InverseInterpolable<T> {
    double inverseInterpolate(T upper, T query);
}
