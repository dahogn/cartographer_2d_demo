
#include <cmath>

#include "pose.h"

Position operator+(const Position &a, const Position &b)
{
    long timestamp = a.timestamp;
    if (b.timestamp > a.timestamp)
    {
        timestamp = b.timestamp;
    }
    return Position(timestamp, a.x + b.x, a.y + b.y, a.theta + b.theta);
}

Position operator-(const Position &a, const Position &b)
{
    long timestamp = a.timestamp;
    if (b.timestamp > a.timestamp)
    {
        timestamp = b.timestamp;
    }
    return Position(timestamp, a.x - b.x, a.y - b.y, a.theta - b.theta);
}

Position operator*(const Position &a, const Position &b)
{
    long timestamp = a.timestamp;
    if (b.timestamp > a.timestamp)
    {
        timestamp = b.timestamp;
    }
    double c = cos(a.theta);
    double s = sin(a.theta);
    double x = b.x * c - b.y * s + a.x;
    double y = b.x * s + b.y * c + a.y;
    return Position(timestamp, x, y, a.theta + b.theta);
}

Position operator/(const Position &p, const Position &a)
{
    long timestamp = a.timestamp;
    if (p.timestamp > a.timestamp)
    {
        timestamp = p.timestamp;
    }
    double x = p.x - a.x;
    double y = p.y - a.y;
    double c = cos(a.theta);
    double s = sin(a.theta);
    return Position(timestamp, x * c + y * s, -x * s + y * c, p.theta - a.theta);
}

bool operator<(const Position &l, const Position &r)
{
    if (l.x != r.x)
    {
        return (l.x < r.x);
    }
    else if (l.y != r.y)
    {
        return (l.y < r.y);
    }
    else
    {
        return (l.theta < r.theta);
    }
}

bool operator==(const Position &a, const Position &b)
{
    return ((a.x == b.x) && (a.y == b.y) && (a.theta == b.theta));
}
