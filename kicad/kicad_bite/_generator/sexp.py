"""Minimal S-expression reader/writer for KiCad files."""
import re

_tok = re.compile(r'\s*(?:(\()|(\))|"((?:[^"\\]|\\.)*)"|([^\s()"]+))')


class Q(str):
    """quoted string atom"""


def parse(text):
    stack = [[]]
    pos = 0
    n = len(text)
    while pos < n:
        m = _tok.match(text, pos)
        if not m:
            if text[pos:].strip() == '':
                break
            raise ValueError(f'bad token at {pos}: {text[pos:pos+40]!r}')
        pos = m.end()
        if m.group(1):
            stack.append([])
        elif m.group(2):
            lst = stack.pop()
            stack[-1].append(lst)
        elif m.group(3) is not None:
            stack[-1].append(Q(re.sub(r'\\(.)', lambda k: {'n': '\n', 't': '\t'}.get(k.group(1), k.group(1)), m.group(3))))
        else:
            stack[-1].append(m.group(4))
    return stack[0]


def dump(x, indent=0):
    sp = '  ' * indent
    if isinstance(x, list):
        if not x:
            return '()'
        simple = all(not isinstance(e, list) for e in x)
        if simple:
            return '(' + ' '.join(dump(e) for e in x) + ')'
        parts = []
        head = []
        for e in x:
            if not isinstance(e, list) and not parts:
                head.append(dump(e))
            else:
                parts.append(e)
        s = '(' + ' '.join(head)
        for e in parts:
            s += '\n' + sp + '  ' + dump(e, indent + 1)
        return s + '\n' + sp + ')'
    if isinstance(x, Q):
        return '"' + x.replace('\\', '\\\\').replace('"', '\\"').replace('\n', '\\n') + '"'
    if isinstance(x, float):
        s = f'{x:.4f}'.rstrip('0').rstrip('.')
        return '0' if s in ('-0', '') else s
    return str(x)


def find(lst, key):
    for e in lst:
        if isinstance(e, list) and e and e[0] == key:
            return e
    return None


def findall(lst, key):
    return [e for e in lst if isinstance(e, list) and e and e[0] == key]
