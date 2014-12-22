Doc Contributions
============

Placeholder

Install Sphinx
-----

Each element in a computer program is either

-   A variable or value literal like ``x``, ``total``, or ``5``
-   A function or computation like the ``+`` in ``x + 1``, the function ``fib``
    in ``fib(3)``, the method ``split`` in ``line.split(',')``, or the ``=`` in
    ``x = 0``
-   Control flow like ``if``, ``for``, or ``return``

Here is a piece of code; see if you can label each term as either
variable/value, function/computation, or control flow

.. code::

    def fib(n):
        a, b = 0, 1
        for i in range(n):
            a, b = b, a + b
        return b
