import math

def yaw_from_quaternion(x, y, z, w):
    """Extract yaw from quaternion (2D orientation)."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)

def yaw_to_direction_vector(yaw):
    """Convert yaw angle to 2D unit direction vector."""
    return [math.cos(yaw), math.sin(yaw)]


def matrix_multiply(A, B):
    """Multiply two matrices A and B."""
    # Check dimensions
    if len(A[0]) != len(B):
        raise ValueError("Number of columns in A must equal number of rows in B")

    result = []
    for i in range(len(A)):  # for each row in A
        row = []
        for j in range(len(B[0])):  # for each column in B
            val = sum(A[i][k] * B[k][j] for k in range(len(B)))
            row.append(val)
        result.append(row)

    return result


def scalar_multiply(scalar, vector):
    """Multiply a vector by a scalar."""
    return [scalar * x for x in vector]


def combine_vectors(v1, v2, operation='+'):
    """Add or subtract two vectors element-wise."""
    if len(v1) != len(v2):
        raise ValueError("Vectors must be of same length")
    if operation == '+':
        return [a + b for a, b in zip(v1, v2)]
    elif operation == '-':
        return [a - b for a, b in zip(v1, v2)]
    else:
        raise ValueError("Unsupported operation. Use '+' or '-'.")


def unit_vector(v):
    """Return the unit vector (normalized vector) of the input vector."""
    magnitude = math.sqrt(sum(x ** 2 for x in v))
    if magnitude == 0:
        raise ValueError("Zero vector has no direction; cannot compute unit vector.")
    return [x / magnitude for x in v]


def angle_between_unit_vectors(u1, u2):
    """Compute the angle in radians between two unit vectors."""
    if len(u1) != len(u2):
        raise ValueError("Vectors must be of same length")
    angle = math.atan2(u2[1], u2[0]) - math.atan2(u1[1], u1[0])

    if angle > math.pi:
        angle -= 2 * math.pi
    elif angle < -1 * math.pi:
        angle += 2 * math.pi

    return angle