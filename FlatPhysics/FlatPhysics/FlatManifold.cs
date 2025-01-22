using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace FlatPhysics
{
    public readonly struct FlatManifold
    {
        public readonly FlatBody bodyA;
        public readonly FlatBody bodyB;
        public readonly FlatVector Normal;
        public readonly float depth;
        public readonly FlatVector Contact1;
        public readonly FlatVector Contact2;
        public readonly int ContactCount;

        public FlatManifold(FlatBody bodyA, FlatBody bodyB, FlatVector normal, float depth, FlatVector contact1, FlatVector contact2, int contactCount)
        { 
            this.bodyA = bodyA;
            this.bodyB = bodyB;
            this.Normal = normal;
            this.depth = depth;
            this.Contact1 = contact1;
            this.Contact2 = contact2;
            this.ContactCount = contactCount;
        }
    }
}
