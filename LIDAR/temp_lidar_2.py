


# Driver code
if __name__ == '__main__':

    polygon1 = [(0, 0), (10, 0), (10, 10), (0, 10)]

    p = (20, 20)
    if (is_inside_polygon(points=polygon1, p=p)):
        print('Yes')
    else:
        print('No')

    p = (5, 5)
    if (is_inside_polygon(points=polygon1, p=p)):
        print('Yes')
    else:
        print('No')

    polygon2 = [(0, 0), (5, 0), (5, 5), (3, 3)]

    p = (3, 3)
    if (is_inside_polygon(points=polygon2, p=p)):
        print('Yes')
    else:
        print('No')

    p = (5, 1)
    if (is_inside_polygon(points=polygon2, p=p)):
        print('Yes')
    else:
        print('No')

    p = (8, 1)
    if (is_inside_polygon(points=polygon2, p=p)):
        print('Yes')
    else:
        print('No')

    polygon3 = [(0, 0), (10, 0), (10, 10), (0, 10)]

    p = (-1, 10)
    if (is_inside_polygon(points=polygon3, p=p)):
        print('Yes')
    else:
        print('No')

# This code is contributed by Vikas Chitturi
