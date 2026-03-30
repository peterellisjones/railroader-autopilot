// Polyfill for .NET Framework 4.8 — enables C# 9+ record types and init-only properties.
// This type is normally provided by the .NET 5+ runtime.
namespace System.Runtime.CompilerServices
{
    internal static class IsExternalInit { }
}
