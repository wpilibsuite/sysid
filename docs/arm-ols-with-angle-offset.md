# Arm OLS with angle offset

If the arm encoder doesn't read zero degrees when the arm is horizontal, the fit
for `Kcos` will be wrong. An angle offset should be added to the model like so.
```
dx/dt = -Kv/Ka x + 1/Ka u - Ks/Ka sgn(x) - Kcos/Ka cos(angle + offset)
```
Use a trig identity to split the cosine into two terms.
```
dx/dt = -Kv/Ka x + 1/Ka u - Ks/Ka sgn(x) - Kcos/Ka (cos(angle) cos(offset) - sin(angle) sin(offset))
dx/dt = -Kv/Ka x + 1/Ka u - Ks/Ka sgn(x) - Kcos/Ka cos(angle) cos(offset) + Kcos/Ka sin(angle) sin(offset)
```
Reorder multiplicands so the offset trig is absorbed by the OLS terms.
```
dx/dt = -Kv/Ka x + 1/Ka u - Ks/Ka sgn(x) - Kcos/Ka cos(offset) cos(angle) + Kcos/Ka sin(offset) sin(angle)
```

## OLS

Let `α = -Kv/Ka`, `β = 1/Ka`, `γ = -Ks/Ka`, `δ = -Kcos/Ka cos(offset)`, and `ε = Kcos/Ka sin(offset)`.
```
dx/dt = αx + βu + γ sgn(x) + δ cos(angle) + ε sin(angle)
```

### Ks, Kv, Ka

Divide the OLS terms by each other to obtain `Ks`, `Kv`, and `Ka`.
```
Ks = -γ/β
Kv = -α/β
Ka = 1/β
```

### Kcos

Take the sum of squares of the OLS terms containing the angle offset. The angle
offset trig functions will form a trig identity that cancels out. Then, just
solve for `Kcos`.
```
δ²+ε² = (-Kcos/Ka cos(offset))² + (Kcos/Ka sin(offset))²
δ²+ε² = (-Kcos/Ka)² cos²(offset) + (Kcos/Ka)² sin²(offset)
δ²+ε² = (Kcos/Ka)² cos²(offset) + (Kcos/Ka)² sin²(offset)
δ²+ε² = (Kcos/Ka)² (cos²(offset) + sin²(offset))
δ²+ε² = (Kcos/Ka)² (1)
δ²+ε² = (Kcos/Ka)²
√(δ²+ε²) = Kcos/Ka
√(δ²+ε²) = Kcos β
Kcos = √(δ²+ε²)/β
```

As a sanity check, when the offset is zero, ε is zero and the equation for
`Kcos` simplifies to -δ/β, the equation previously used by SysId.

### Angle offset

Divide ε by δ, combine the trig functions into `tan(offset)`, then use `atan2()`
to preserve the angle quadrant. Maintaining the proper negative signs in the
numerator and denominator are important for obtaining the correct result.
```
δ = -kcos/ka cos(offset)
ε = kcos/ka sin(offset)
sin(offset)/-cos(offset) = ε/δ
sin(offset)/cos(offset) = ε/-δ
tan(offset) = ε/-δ
offset = atan2(ε, -δ)
```
