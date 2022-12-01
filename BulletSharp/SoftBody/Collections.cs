using System;
using System.Collections.Generic;
using static BulletSharp.UnsafeNativeMethods;

namespace BulletSharp.SoftBody
{
	public class NodePtrArrayEnumerator : IEnumerator<SoftBodyNode>
	{
		private int _i;
		private int _count;
		private IList<SoftBodyNode> _array;

		public NodePtrArrayEnumerator(IList<SoftBodyNode> array)
		{
			_array = array;
			_count = array.Count;
			_i = -1;
		}

		public void Dispose()
		{
		}

		public bool MoveNext()
		{
			_i++;
			return _i != _count;
		}

		public void Reset()
		{
			_i = 0;
		}

		public SoftBodyNode Current => _array[_i];

		object System.Collections.IEnumerator.Current => _array[_i];
	}

	public class NodePtrArray : FixedSizeArray<SoftBodyNode>, IList<SoftBodyNode>
	{
		internal NodePtrArray(IntPtr native, int count)
			: base(native, count)
		{
		}

		public int IndexOf(SoftBodyNode item)
		{
			throw new NotImplementedException();
		}

		public SoftBodyNode this[int index]
		{
			get
			{
				if ((uint)index >= (uint)Count)
				{
					throw new ArgumentOutOfRangeException(nameof(index));
				}
				return new SoftBodyNode(btSoftBodyNodePtrArray_at(Native, index));
			}
			set
			{
				btSoftBodyNodePtrArray_set(Native, value.Native, index);
			}
		}

		public bool Contains(SoftBodyNode item)
		{
			throw new NotImplementedException();
		}

		public void CopyTo(SoftBodyNode[] array, int arrayIndex)
		{
			throw new NotImplementedException();
		}

		public IEnumerator<SoftBodyNode> GetEnumerator()
		{
			return new NodePtrArrayEnumerator(this);
		}

		System.Collections.IEnumerator System.Collections.IEnumerable.GetEnumerator()
		{
			return new NodePtrArrayEnumerator(this);
		}
	}
}
